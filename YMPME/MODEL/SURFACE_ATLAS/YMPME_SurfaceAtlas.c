#include "YMPME_SurfaceAtlas.h"
#include <limits.h>
#include <string.h>
static int less(pme_surface_atlas_edge a, pme_surface_atlas_edge b)
{
	return a.a != b.a		  ? a.a < b.a
		   : a.b != b.b		  ? a.b < b.b
		   : a.face != b.face ? a.face < b.face
							  : a.slot < b.slot;
}
static void sift(pme_surface_atlas_edge *e, size_t root, size_t n)
{
	while (root < n / 2)
	{
		size_t child = root * 2 + 1;
		if (child + 1 < n && less(e[child], e[child + 1]))
			child++;
		if (!less(e[root], e[child]))
			break;
		pme_surface_atlas_edge t = e[root];
		e[root] = e[child];
		e[child] = t;
		root = child;
	}
}
static void sort(pme_surface_atlas_edge *e, size_t n)
{
	for (size_t i = n / 2; i > 0; i--)
		sift(e, i - 1, n);
	for (size_t i = n; i > 1; i--)
	{
		pme_surface_atlas_edge t = e[0];
		e[0] = e[i - 1];
		e[i - 1] = t;
		sift(e, 0, i - 1);
	}
}
pme_surface_atlas_desc YMPME_SurfaceAtlas_DefaultDesc(void)
{
	return (pme_surface_atlas_desc){1e-6f, .35f, 128};
}
static void bounds(pme_surface_atlas_chart *b, const pme_surface_atlas_face *f)
{
	for (int k = 0; k < 3; k++)
	{
		float x = f->xy[k][0] + .5f * f->xy[k][1], y = .866025403784f * f->xy[k][1];
		b->lowX = fminf(b->lowX, x);
		b->highX = fmaxf(b->highX, x);
		b->lowY = fminf(b->lowY, y);
		b->highY = fmaxf(b->highY, y);
	}
}
pme_result YMPME_SurfaceAtlas_Build(const pme_surface_atlas_desc *d, const pme_vec3 *v, size_t nv,
									const uint32_t *index, size_t nf, const uint32_t *regions,
									pme_surface_atlas_workspace *w, float *uv, uint32_t *ids,
									size_t cap, pme_surface_atlas_result *out)
{
	if (!d || !w || !out || !v || !index || !uv || !ids || !nv || !nf || nv > UINT32_MAX ||
		nf > INT32_MAX / 16 || !isfinite(d->weldTolerance) || d->weldTolerance < 0 ||
		!isfinite(d->padding) || d->padding < 0 || d->padding > 1000)
		return PME_INVALID_ARGUMENT;
	if (cap < nf || w->vertexCapacity < nv || w->faceCapacity < nf)
		return PME_CAPACITY_EXCEEDED;
	if (!w->canonical || !w->queue || !w->edges || !w->faces || !w->charts)
		return PME_INVALID_ARGUMENT;
	for (size_t i = 0; i < nv; i++)
		if (!YMPME_VecFinite(v[i]))
			return PME_INVALID_ARGUMENT;
	for (size_t f = 0; f < nf; f++)
	{
		for (int k = 0; k < 3; k++)
			if (index[f * 3 + k] >= nv)
				return PME_INVALID_ARGUMENT;
		pme_vec3 a = YMPME_VecSub(v[index[f * 3 + 1]], v[index[f * 3]]),
				 b = YMPME_VecSub(v[index[f * 3 + 2]], v[index[f * 3]]);
		pme_vec3 c = YMPME_VecCross(a, b);
		float area2 = YMPME_VecDot(c, c);
		if (!isfinite(area2) || area2 <= 0)
			return PME_INVALID_ARGUMENT;
	}
	double tolerance = (double)d->weldTolerance * d->weldTolerance;
	for (size_t i = 0; i < nv; i++)
	{
		w->canonical[i] = (uint32_t)i;
		for (size_t j = 0; j < i; j++)
		{
			double x = (double)v[i].x - v[j].x, y = (double)v[i].y - v[j].y,
				   z = (double)v[i].z - v[j].z;
			if (x * x + y * y + z * z <= tolerance)
			{
				w->canonical[i] = w->canonical[j];
				break;
			}
		}
	}
	memset(w->faces, 0, nf * sizeof(*w->faces));
	for (size_t f = 0; f < nf; f++)
	{
		uint32_t canonical[3];
		for (int k = 0; k < 3; k++)
			canonical[k] = w->canonical[index[f * 3 + k]];
		if (canonical[0] == canonical[1] || canonical[1] == canonical[2] ||
			canonical[2] == canonical[0])
			return PME_INVALID_ARGUMENT;
		w->faces[f].chart = -1;
		for (int k = 0; k < 3; k++)
		{
			uint32_t a = canonical[k], b = canonical[(k + 1) % 3];
			w->faces[f].neighbour[k] = -1;
			w->edges[f * 3 + k] =
				(pme_surface_atlas_edge){a < b ? a : b, a < b ? b : a, (uint32_t)f, (uint32_t)k};
		}
	}
	sort(w->edges, nf * 3);
	pme_surface_atlas_result r = {0};
	for (size_t i = 0; i < nf * 3;)
	{
		size_t end = i + 1;
		while (end < nf * 3 && w->edges[i].a == w->edges[end].a && w->edges[i].b == w->edges[end].b)
			end++;
		if (end - i == 2)
		{
			pme_surface_atlas_edge a = w->edges[i], b = w->edges[i + 1];
			if (!regions || regions[a.face] == regions[b.face])
			{
				w->faces[a.face].neighbour[a.slot] = (int32_t)b.face;
				w->faces[b.face].neighbour[b.slot] = (int32_t)a.face;
				r.adjacentEdges++;
			}
		}
		else if (end - i > 2)
			r.nonManifoldEdges++;
		i = end;
	}
	for (size_t seed = 0; seed < nf; seed++)
		if (w->faces[seed].chart < 0)
		{
			uint32_t chart = (uint32_t)r.charts++;
			pme_surface_atlas_chart *b = &w->charts[chart];
			*b = (pme_surface_atlas_chart){INFINITY, INFINITY, -INFINITY, -INFINITY, 0, 0, 0, 0};
			pme_surface_atlas_face *first = &w->faces[seed];
			first->chart = (int32_t)chart;
			first->xy[0][0] = 0;
			first->xy[0][1] = 0;
			first->xy[1][0] = 1;
			first->xy[1][1] = 0;
			first->xy[2][0] = 0;
			first->xy[2][1] = 1;
			size_t head = 0, tail = 1;
			w->queue[0] = (uint32_t)seed;
			bounds(b, first);
			while (head < tail)
			{
				uint32_t at = w->queue[head++];
				pme_surface_atlas_face *f = &w->faces[at];
				for (int edge = 0; edge < 3; edge++)
				{
					int next = f->neighbour[edge];
					if (next < 0 || w->faces[next].chart >= 0 ||
						(d->maxFacesPerChart && tail >= d->maxFacesPerChart))
						continue;
					pme_surface_atlas_face candidate = w->faces[next];
					uint32_t a = w->canonical[index[at * 3 + edge]],
							 c = w->canonical[index[at * 3 + (edge + 1) % 3]];
					for (int k = 0; k < 3; k++)
					{
						uint32_t q = w->canonical[index[next * 3 + k]];
						for (int axis = 0; axis < 2; axis++)
							candidate.xy[k][axis] = q == a	 ? f->xy[edge][axis]
													: q == c ? f->xy[(edge + 1) % 3][axis]
															 : f->xy[edge][axis] +
																   f->xy[(edge + 1) % 3][axis] -
																   f->xy[(edge + 2) % 3][axis];
					}
					int32_t x = candidate.xy[0][0] + candidate.xy[1][0] + candidate.xy[2][0],
							y = candidate.xy[0][1] + candidate.xy[1][1] + candidate.xy[2][1];
					int occupied = 0;
					for (size_t q = 0; q < tail; q++)
					{
						pme_surface_atlas_face *p = &w->faces[w->queue[q]];
						if (x == p->xy[0][0] + p->xy[1][0] + p->xy[2][0] &&
							y == p->xy[0][1] + p->xy[1][1] + p->xy[2][1])
						{
							occupied = 1;
							break;
						}
					}
					if (occupied)
						continue;
					candidate.chart = (int32_t)chart;
					w->faces[next] = candidate;
					w->queue[tail++] = (uint32_t)next;
					bounds(b, &candidate);
				}
			}
			b->count = (uint32_t)tail;
		}
	float area = 0, maxWidth = 0;
	for (size_t c = 0; c < r.charts; c++)
	{
		pme_surface_atlas_chart *b = &w->charts[c];
		float width = b->highX - b->lowX + 2 * d->padding,
			  height = b->highY - b->lowY + 2 * d->padding;
		area += width * height;
		maxWidth = fmaxf(maxWidth, width);
	}
	float shelfWidth = fmaxf(maxWidth, sqrtf(area) * 1.15f), x = 0, y = 0, row = 0, totalWidth = 0;
	for (size_t c = 0; c < r.charts; c++)
	{
		size_t selected = r.charts;
		float height = -1;
		for (size_t j = 0; j < r.charts; j++)
			if (!w->charts[j].packed && w->charts[j].highY - w->charts[j].lowY > height)
			{
				selected = j;
				height = w->charts[j].highY - w->charts[j].lowY;
			}
		pme_surface_atlas_chart *b = &w->charts[selected];
		b->packed = 1;
		float width = b->highX - b->lowX + 2 * d->padding;
		height += 2 * d->padding;
		if (x > 0 && x + width > shelfWidth)
		{
			y += row;
			x = 0;
			row = 0;
		}
		b->x = x + d->padding - b->lowX;
		b->y = y + d->padding - b->lowY;
		x += width;
		row = fmaxf(row, height);
		totalWidth = fmaxf(totalWidth, x);
	}
	float scale = 1 / fmaxf(totalWidth, y + row);
	r.triangleArea = .433012701892f * scale * scale;
	r.occupancy = r.triangleArea * nf;
	for (size_t f = 0; f < nf; f++)
	{
		pme_surface_atlas_face *a = &w->faces[f];
		pme_surface_atlas_chart *b = &w->charts[a->chart];
		ids[f] = (uint32_t)a->chart;
		for (int k = 0; k < 3; k++)
		{
			uv[f * 6 + k * 2] = (a->xy[k][0] + .5f * a->xy[k][1] + b->x) * scale;
			uv[f * 6 + k * 2 + 1] = (.866025403784f * a->xy[k][1] + b->y) * scale;
		}
		for (int e = 0; e < 3; e++)
		{
			int next = a->neighbour[e];
			if (next < 0 || (size_t)next < f || w->faces[next].chart != a->chart)
				continue;
			int matches = 0;
			for (int k = 0; k < 3; k++)
				for (int l = 0; l < 3; l++)
					if (w->canonical[index[f * 3 + k]] == w->canonical[index[next * 3 + l]] &&
						a->xy[k][0] == w->faces[next].xy[l][0] &&
						a->xy[k][1] == w->faces[next].xy[l][1])
						matches++;
			if (matches == 2)
				r.joinedEdges++;
		}
	}
	*out = r;
	return PME_OK;
}
