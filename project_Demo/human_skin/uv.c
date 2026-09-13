#include "uv.h"
#include <math.h>
#include <string.h>
static double edge(double ax, double ay, double bx, double by, double x, double y)
{
	return (bx - ax) * (y - ay) - (by - ay) * (x - ax);
}
int HumanUV_Init(human_uv_map *m, const float *corners, size_t count)
{
	if (!m || !corners || count > UV_MAX_FACES)
		return 0;
	for (size_t i = 0; i < count * 6; i++)
		if (!isfinite(corners[i]) || corners[i] < 0 || corners[i] > 1)
			return 0;
	memset(m, 0, sizeof(*m));
	memset(m->patch, 255, sizeof m->patch);
	m->count = count;
	for (size_t i = 0; i < count; ++i)
	{
		double x[3], y[3];
		for (int j = 0; j < 3; ++j)
		{
			x[j] = corners[i * 6 + j * 2] * UV_SIZE;
			y[j] = corners[i * 6 + j * 2 + 1] * UV_SIZE;
		}
		double area = edge(x[0], y[0], x[1], y[1], x[2], y[2]);
		if (fabs(area) < 1e-8f)
			continue;
		int x0 = (int)fmax(0, floor(fmin(x[0], fmin(x[1], x[2])))),
			x1 = (int)fmin(UV_SIZE - 1, ceil(fmax(x[0], fmax(x[1], x[2]))));
		int y0 = (int)fmax(0, floor(fmin(y[0], fmin(y[1], y[2])))),
			y1 = (int)fmin(UV_SIZE - 1, ceil(fmax(y[0], fmax(y[1], y[2]))));
		for (int py = y0; py <= y1; ++py)
			for (int px = x0; px <= x1; ++px)
			{
				double a = edge(x[1], y[1], x[2], y[2], px + .5f, py + .5f) / area;
				double b = edge(x[2], y[2], x[0], y[0], px + .5f, py + .5f) / area, c = 1 - a - b;
				if (a < -1e-6f || b < -1e-6f || c < -1e-6f)
					continue;
				size_t at = (size_t)py * UV_SIZE + px;
				int inside = a > 1e-5f && b > 1e-5f && c > 1e-5f;
				if (m->patch[at] != UV_EMPTY && m->interior[at] && inside && !m->overlap[at])
				{
					m->overlap[at] = 1;
					++m->overlapPixels;
				}
				if (m->patch[at] == UV_EMPTY || (!m->interior[at] && inside))
				{
					m->patch[at] = (uint16_t)i;
					m->interior[at] = (uint8_t)inside;
				}
			}
	}
	uint8_t covered[UV_MAX_FACES] = {0};
	for (size_t i = 0; i < UV_SIZE * UV_SIZE; ++i)
		if (m->patch[i] != UV_EMPTY && !m->overlap[i])
			covered[m->patch[i]] = 1;
	for (size_t i = 0; i < count; ++i)
		if (!covered[i])
			++m->uncoveredPatches;
	return 1;
}
int HumanUV_Pick(const human_uv_map *m, int x, int y)
{
	if (!m || x < 0 || y < 0 || x >= UV_SIZE || y >= UV_SIZE)
		return -1;
	size_t at = (size_t)y * UV_SIZE + x;
	if (m->overlap[at])
		return -2;
	return m->patch[at] == UV_EMPTY ? -1 : (int)m->patch[at];
}

void HumanUV_Render(const human_uv_map *m, GRE_RenderTarget t, const GRErgb24 *colors, int selected,
					float centerU, float centerV, float zoom)
{
	if (!m || !t || !colors || !isfinite(zoom) || zoom < 1)
		return;
	for (int y = 0; y < t->height; y++)
		for (int x = 0; x < t->width; x++)
		{
			int px = (int)((centerU + ((x + .5f) / t->width - .5f) / zoom) * UV_SIZE);
			int py = (int)((centerV + ((y + .5f) / t->height - .5f) / zoom) * UV_SIZE);
			GRErgb24 c = {15, 21, 31};
			if (px >= 0 && py >= 0 && px < UV_SIZE && py < UV_SIZE)
			{
				size_t at = (size_t)py * UV_SIZE + px;
				unsigned p = m->patch[at];
				if (m->overlap[at])
					c = (GRErgb24){240, 50, 220};
				else if (p != UV_EMPTY)
				{
					c = colors[p];
					if ((int)p == selected &&
						(px == 0 || py == 0 || px == UV_SIZE - 1 || py == UV_SIZE - 1 ||
						 m->patch[at - 1] != p || m->patch[at + 1] != p ||
						 m->patch[at - UV_SIZE] != p || m->patch[at + UV_SIZE] != p))
						c = (GRErgb24){255, 245, 180};
				}
			}
			t->data[y * t->width + x] = GRE_FramePixel_From_RGB24(c);
		}
}
