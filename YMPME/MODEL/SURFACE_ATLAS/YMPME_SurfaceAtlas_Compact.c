#include "YMPME_SurfaceAtlas.h"
#include <string.h>
/* A bitmap cell conservatively covers its entire square, including chart padding. */
static void silhouette(const pme_surface_atlas_workspace *w, size_t nf, size_t chart, unsigned res,
					   unsigned stride, uint64_t *mask, float margin)
{
	const pme_surface_atlas_chart *b = &w->charts[chart];
	for (size_t f = 0; f < nf; f++)
		if (w->faces[f].chart == (int32_t)chart)
		{
			float x[3], y[3], loX = INFINITY, loY = INFINITY, hiX = -INFINITY, hiY = -INFINITY;
			for (int k = 0; k < 3; k++)
			{
				x[k] = w->faces[f].xy[k][0] + .5f * w->faces[f].xy[k][1] - b->lowX + margin;
				y[k] = .866025403784f * w->faces[f].xy[k][1] - b->lowY + margin;
				loX = fminf(loX, x[k]);
				hiX = fmaxf(hiX, x[k]);
				loY = fminf(loY, y[k]);
				hiY = fmaxf(hiY, y[k]);
			}
			float sign =
				((x[1] - x[0]) * (y[2] - y[0]) - (y[1] - y[0]) * (x[2] - x[0])) > 0 ? 1 : -1;
			int x0 = (int)floorf((loX - margin) * res), x1 = (int)ceilf((hiX + margin) * res);
			int y0 = (int)floorf((loY - margin) * res), y1 = (int)ceilf((hiY + margin) * res);
			for (int py = y0; py < y1; py++)
				for (int px = x0; px < x1; px++)
				{
					float u = (px + .5f) / res, v = (py + .5f) / res;
					int inside = 1;
					for (int k = 0; k < 3; k++)
					{
						int j = (k + 1) % 3;
						float distance =
							sign * ((x[j] - x[k]) * (v - y[k]) - (y[j] - y[k]) * (u - x[k]));
						if (distance < -margin)
						{
							inside = 0;
							break;
						}
					}
					if (inside && px >= 0 && py >= 0)
						mask[(size_t)py * stride + (unsigned)px / 64] |= UINT64_C(1) << (px % 64);
				}
		}
}
static int place(uint64_t *occupied, const uint64_t *shape, unsigned stride, unsigned width,
				 unsigned height, unsigned x, unsigned y, int commit)
{
	unsigned word = x / 64, shift = x % 64, nw = (width + 63) / 64;
	for (unsigned row = 0; row < height; row++)
		for (unsigned k = 0; k < nw; k++)
		{
			uint64_t bits = shape[(size_t)row * stride + k];
			if (!bits)
				continue;
			size_t at = (size_t)(row + y) * stride + word + k;
			uint64_t a = bits << shift, b = shift ? bits >> (64 - shift) : 0;
			if (commit)
			{
				occupied[at] |= a;
				if (b)
					occupied[at + 1] |= b;
			}
			else if ((occupied[at] & a) || (b && (occupied[at + 1] & b)))
				return 0;
		}
	return 1;
}
pme_result YMPME_SurfaceAtlas_Compact(const pme_surface_atlas_desc *d,
									  pme_surface_atlas_workspace *w, size_t nf, uint32_t side,
									  uint32_t res, uint64_t *occupied, uint64_t *shape,
									  size_t capacity, float *uv, pme_surface_atlas_result *out)
{
	if (!d || !w || !out || !uv || !occupied || !shape || occupied == shape || !w->faces ||
		!w->charts || !w->queue || !nf || nf > w->faceCapacity || !out->charts ||
		out->charts > nf || side < 8 || side > 4096 || res < 2 || res > 32 ||
		!isfinite(d->padding) || d->padding < 0 || !isfinite(out->triangleArea) ||
		out->triangleArea <= 0)
		return PME_INVALID_ARGUMENT;
	unsigned stride = (side + 63) / 64;
	size_t words = (size_t)side * stride;
	if (capacity < words)
		return PME_CAPACITY_EXCEEDED;
	/* Extra half-diagonal protects actual triangles between raster samples. */
	float margin = d->padding + .707107f / res;
	unsigned maxWidth = 0, maxHeight = 0;
	for (size_t c = 0; c < out->charts; c++)
	{
		pme_surface_atlas_chart *b = &w->charts[c];
		float width = (b->highX - b->lowX + 2 * margin) * res,
			  height = (b->highY - b->lowY + 2 * margin) * res;
		if (!isfinite(width + height) || width <= 0 || height <= 0)
			return PME_INVALID_ARGUMENT;
		if (width > side || height > side)
			return PME_CAPACITY_EXCEEDED;
		maxWidth = (unsigned)fmaxf(maxWidth, ceilf(width));
		maxHeight = (unsigned)fmaxf(maxHeight, ceilf(height));
		b->packed = 0;
	}
	for (size_t f = 0; f < nf; f++)
		if (w->faces[f].chart < 0 || (size_t)w->faces[f].chart >= out->charts)
			return PME_INVALID_ARGUMENT;
	/* Larger outlines first; face IDs, seams and local chart orientation stay fixed. */
	for (size_t i = 0; i < out->charts; i++)
	{
		size_t best = 0;
		float score = -1;
		for (size_t c = 0; c < out->charts; c++)
			if (!w->charts[c].packed)
			{
				pme_surface_atlas_chart *b = &w->charts[c];
				float a = (b->highX - b->lowX) * (b->highY - b->lowY);
				if (a > score)
				{
					score = a;
					best = c;
				}
			}
		w->queue[i] = (uint32_t)best;
		w->charts[best].packed = 1;
	}
	float bestExtent = sqrtf(.433012701892f / out->triangleArea) * res;
	unsigned start = (unsigned)ceilf(sqrtf(nf * .433012701892f / .72f) * res);
	start = start > maxWidth ? start : maxWidth;
	unsigned limit = (unsigned)fminf(side, ceilf(bestExtent));
	for (unsigned width = start; width <= limit; width += res * 2)
	{
		memset(occupied, 0, words * sizeof(*occupied));
		unsigned usedW = 0, usedH = 0;
		int fit = 1;
		for (size_t i = 0; i < out->charts; i++)
		{
			size_t c = w->queue[i];
			pme_surface_atlas_chart *b = &w->charts[c];
			unsigned cw = (unsigned)ceilf((b->highX - b->lowX + 2 * margin) * res);
			unsigned ch = (unsigned)ceilf((b->highY - b->lowY + 2 * margin) * res);
			memset(shape, 0, words * sizeof(*shape));
			silhouette(w, nf, c, res, stride, shape, margin);
			int found = 0;
			unsigned heightLimit = (unsigned)fminf(side, ceilf(bestExtent));
			for (unsigned y = 0; y + ch <= heightLimit && !found; y++)
				for (unsigned x = 0; x + cw <= width; x++)
				{
					if (!place(occupied, shape, stride, cw, ch, x, y, 0))
						continue;
					place(occupied, shape, stride, cw, ch, x, y, 1);
					b->x = (float)x / res + margin - b->lowX;
					b->y = (float)y / res + margin - b->lowY;
					usedW = usedW > x + cw ? usedW : x + cw;
					usedH = usedH > y + ch ? usedH : y + ch;
					found = 1;
					break;
				}
			if (!found)
			{
				fit = 0;
				break;
			}
		}
		float extent = (float)(usedW > usedH ? usedW : usedH);
		if (fit && extent < bestExtent)
		{
			bestExtent = extent;
			float scale = res / extent;
			for (size_t f = 0; f < nf; f++)
			{
				pme_surface_atlas_face *a = &w->faces[f];
				pme_surface_atlas_chart *b = &w->charts[a->chart];
				for (int k = 0; k < 3; k++)
				{
					uv[f * 6 + k * 2] = (a->xy[k][0] + .5f * a->xy[k][1] + b->x) * scale;
					uv[f * 6 + k * 2 + 1] = (.866025403784f * a->xy[k][1] + b->y) * scale;
				}
			}
			out->triangleArea = .433012701892f * scale * scale;
			out->occupancy = out->triangleArea * nf;
			limit = (unsigned)fminf(limit, ceilf(bestExtent));
		}
	}
	/* Restore chart offsets matching the retained output, even if the last trial failed. */
	float invScale = sqrtf(.433012701892f / out->triangleArea);
	for (size_t f = 0; f < nf; f++)
	{
		pme_surface_atlas_face *a = &w->faces[f];
		pme_surface_atlas_chart *b = &w->charts[a->chart];
		b->x = uv[f * 6] * invScale - a->xy[0][0] - .5f * a->xy[0][1];
		b->y = uv[f * 6 + 1] * invScale - .866025403784f * a->xy[0][1];
	}
	(void)maxHeight;
	return PME_OK;
}
