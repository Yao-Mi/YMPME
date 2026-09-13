#include "uv_map.h"
#include <string.h>
static double edge(double ax, double ay, double bx, double by, double x, double y)
{
	return (bx - ax) * (y - ay) - (by - ay) * (x - ax);
}
static int init(hand_uv_map *m, const unsigned *faces, size_t count, const float *corners)
{
	if (!m || !faces || count > HAND_FACES)
		return 0;
	for (size_t i = 0; i < count; ++i)
	{
		if (faces[i] >= HAND_FACES)
			return 0;
		for (int j = 0; j < 3; ++j)
		{
			unsigned v = handTriangles[faces[i]][j];
			float u = corners ? corners[i * 6 + j * 2] : handUV[v][0];
			float w = corners ? corners[i * 6 + j * 2 + 1] : handUV[v][1];
			if (v >= HAND_POINTS || !isfinite(u) || !isfinite(w) || u < 0 || u > 1 || w < 0 ||
				w > 1)
				return 0;
		}
	}
	memset(m, 0, sizeof(*m));
	memset(m->patch, 255, sizeof m->patch);
	m->count = count;
	for (size_t i = 0; i < count; ++i)
	{
		m->face[i] = faces[i];
		const uint16_t *t = handTriangles[faces[i]];
		double x[3], y[3];
		for (int j = 0; j < 3; ++j)
		{
			x[j] = (corners ? corners[i * 6 + j * 2] : handUV[t[j]][0]) * HAND_UV_SIZE;
			y[j] = (corners ? corners[i * 6 + j * 2 + 1] : handUV[t[j]][1]) * HAND_UV_SIZE;
		}
		double area = edge(x[0], y[0], x[1], y[1], x[2], y[2]);
		if (fabs(area) < 1e-8f)
			continue;
		int x0 = (int)fmax(0, floor(fmin(x[0], fmin(x[1], x[2])))),
			x1 = (int)fmin(HAND_UV_SIZE - 1, ceil(fmax(x[0], fmax(x[1], x[2]))));
		int y0 = (int)fmax(0, floor(fmin(y[0], fmin(y[1], y[2])))),
			y1 = (int)fmin(HAND_UV_SIZE - 1, ceil(fmax(y[0], fmax(y[1], y[2]))));
		for (int py = y0; py <= y1; ++py)
			for (int px = x0; px <= x1; ++px)
			{
				double a = edge(x[1], y[1], x[2], y[2], px + .5f, py + .5f) / area;
				double b = edge(x[2], y[2], x[0], y[0], px + .5f, py + .5f) / area, c = 1 - a - b;
				if (a < -1e-6f || b < -1e-6f || c < -1e-6f)
					continue;
				size_t at = (size_t)py * HAND_UV_SIZE + px;
				int inside = a > 1e-5f && b > 1e-5f && c > 1e-5f;
				if (m->patch[at] != HAND_UV_EMPTY && m->interior[at] && inside && !m->overlap[at])
				{
					m->overlap[at] = 1;
					++m->overlapPixels;
				}
				if (m->patch[at] == HAND_UV_EMPTY || (!m->interior[at] && inside))
				{
					m->patch[at] = (uint16_t)i;
					m->interior[at] = (uint8_t)inside;
				}
			}
	}
	uint8_t covered[HAND_FACES] = {0};
	for (size_t i = 0; i < HAND_UV_SIZE * HAND_UV_SIZE; ++i)
		if (m->patch[i] != HAND_UV_EMPTY && !m->overlap[i])
			covered[m->patch[i]] = 1;
	for (size_t i = 0; i < count; ++i)
		if (!covered[i])
			++m->uncoveredPatches;
	return 1;
}
int HandUV_Pick(const hand_uv_map *m, int x, int y)
{
	if (!m || x < 0 || y < 0 || x >= HAND_UV_SIZE || y >= HAND_UV_SIZE)
		return -1;
	size_t at = (size_t)y * HAND_UV_SIZE + x;
	if (m->overlap[at])
		return -2;
	return m->patch[at] == HAND_UV_EMPTY ? -1 : (int)m->patch[at];
}
GRErgb24 HandUV_Color(float value, float maximum)
{
	float t = YMPME_Clamp(value / maximum, 0, 1);
	return (GRErgb24){(uint8)(35 + 220 * t), (uint8)(100 + 110 * (1 - fabsf(2 * t - 1))),
					  (uint8)(170 * (1 - t) + 30)};
}
static void render(const hand_uv_map *m, GRE_RenderTarget target, const pme_surface_taxel *cells,
				   int forceMode, int selected, const GRErgb24 *colors)
{
	if (!m || !target || (!cells && !colors) || target->width != HAND_UV_SIZE ||
		target->height != HAND_UV_SIZE)
		return;
	/* 每个像素只查初始化缓存，不逐帧重展开/光栅化，也不将像素当作物理面积。 */
	for (int y = 0; y < HAND_UV_SIZE; ++y)
		for (int x = 0; x < HAND_UV_SIZE; ++x)
		{
			size_t at = (size_t)y * HAND_UV_SIZE + x;
			uint16_t p = m->patch[at];
			GRErgb24 color = {15, 21, 31};
			if (m->overlap[at])
				color = (GRErgb24){240, 50, 220};
			else if (p != HAND_UV_EMPTY)
			{
				color = colors
							? colors[p]
							: HandUV_Color(forceMode == 2
											   ? sqrtf(YMPME_VecDot(cells[p].force, cells[p].force))
										   : forceMode ? cells[p].normalForce
													   : cells[p].pressure,
										   forceMode ? .1f : 1000);
				int boundary = x == 0 || y == 0 || x == HAND_UV_SIZE - 1 || y == HAND_UV_SIZE - 1 ||
							   m->patch[at - 1] != p || m->patch[at + 1] != p ||
							   m->patch[at - HAND_UV_SIZE] != p || m->patch[at + HAND_UV_SIZE] != p;
				if ((int)p == selected && boundary)
					color = (GRErgb24){255, 245, 170};
				else if ((x > 0 && m->patch[at - 1] == HAND_UV_EMPTY) ||
						 (y > 0 && m->patch[at - HAND_UV_SIZE] == HAND_UV_EMPTY))
					color = (GRErgb24){100, 155, 200};
			}
			target->data[at] = GRE_FramePixel_From_RGB24(color);
		}
}

int HandUV_Init(hand_uv_map *m, const unsigned *faces, size_t count)
{
	return init(m, faces, count, NULL);
}
int HandUV_InitCorners(hand_uv_map *m, const unsigned *faces, size_t count, const float *uv)
{
	return uv && init(m, faces, count, uv);
}

void HandUV_Render(const hand_uv_map *m, GRE_RenderTarget t, const pme_surface_taxel *c, int mode,
				   int selected)
{
	render(m, t, c, mode, selected, NULL);
}
void HandUV_RenderColors(const hand_uv_map *m, GRE_RenderTarget t, const GRErgb24 *colors,
						 int selected)
{
	render(m, t, NULL, 0, selected, colors);
}
