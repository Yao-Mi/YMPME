#include "YMPME_Terrain.h"
#include <string.h>
static float bump(float x, float z, float cx, float cz, float rx, float rz, float h)
{
	float u = (x - cx) / rx, v = (z - cz) / rz, r = sqrtf(u * u + v * v);
	return r < 1 ? h * 0.5f * (1 + cosf(3.14159265359f * r)) : 0;
}
void YMPME_Terrain_InitHills(pme_terrain *t)
{
	if (!t)
		return;
	int x, z;
	for (z = 0; z < PME_TERRAIN_N; ++z)
		for (x = 0; x < PME_TERRAIN_N; ++x)
		{
			float px = PME_TERRAIN_MIN + x * PME_TERRAIN_STEP, pz = PME_TERRAIN_MIN + z * PME_TERRAIN_STEP;
			float h = bump(px, pz, -4, 0, 2.1f, 2.7f, 0.65f) + bump(px, pz, -4, 5, 2.5f, 2.5f, 0.85f);
			h += bump(px, pz, 0, 0, 1.4f, 1.8f, 0.28f) + bump(px, pz, 0, 3.5f, 1.4f, 1.8f, 0.36f);
			h += bump(px, pz, 4.65f, -1, 0.9f, 1.5f, 0.48f) + bump(px, pz, 3.35f, 2, 0.9f, 1.5f, 0.48f);
			h += bump(px, pz, 4.0f, 5, 2.3f, 2.3f, 0.8f);
			t->height[z * PME_TERRAIN_N + x] = h;
		}
}
int YMPME_Terrain_Sample(const pme_terrain *t, float x, float z, float *height, pme_vec3 *normal)
{
	float gx, gz, u, v, a, b, c, d, h, dx, dz;
	int ix, iz;
	if (!t || !height || !isfinite(x) || !isfinite(z))
		return 0;
	gx = (x - PME_TERRAIN_MIN) / PME_TERRAIN_STEP;
	gz = (z - PME_TERRAIN_MIN) / PME_TERRAIN_STEP;
	if (gx < 0 || gz < 0 || gx > PME_TERRAIN_N - 1 || gz > PME_TERRAIN_N - 1)
		return 0;
	ix = (int)gx;
	iz = (int)gz;
	if (ix == PME_TERRAIN_N - 1)
		--ix;
	if (iz == PME_TERRAIN_N - 1)
		--iz;
	u = gx - ix;
	v = gz - iz;
	a = t->height[iz * PME_TERRAIN_N + ix];
	b = t->height[iz * PME_TERRAIN_N + ix + 1];
	c = t->height[(iz + 1) * PME_TERRAIN_N + ix];
	d = t->height[(iz + 1) * PME_TERRAIN_N + ix + 1];
	/* GRE RectPlane 将单元拆为 (00,01,10) 和 (10,01,11)。 */
	if (u + v <= 1)
	{
		h = a + u * (b - a) + v * (c - a);
		dx = (b - a) / PME_TERRAIN_STEP;
		dz = (c - a) / PME_TERRAIN_STEP;
	}
	else
	{
		h = d + (1 - v) * (b - d) + (1 - u) * (c - d);
		dx = (d - c) / PME_TERRAIN_STEP;
		dz = (d - b) / PME_TERRAIN_STEP;
	}
	*height = h;
	if (normal)
		*normal = YMPME_VecScale(YMPME_Vec3(-dx, 1, -dz), 1 / sqrtf(dx * dx + 1 + dz * dz));
	return 1;
}
