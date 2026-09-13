#include "force_channels.h"
#define V YMPME_Vec3
static float length(pme_vec3 v) { return sqrtf(YMPME_VecDot(v, v)); }
int DemoForce_Local(const pme_vec3 p[3], const float uv[6], pme_vec3 n, pme_vec3 f, pme_vec3 *out)
{
	if (!p || !uv || !out || !YMPME_VecFinite(n) || !YMPME_VecFinite(f))
		return 0;
	for (int i = 0; i < 3; i++)
		if (!YMPME_VecFinite(p[i]))
			return 0;
	for (int i = 0; i < 6; i++)
		if (!isfinite(uv[i]))
			return 0;
	if (fabsf(YMPME_VecDot(n, n) - 1) > 1e-4f)
		return 0;
	float du1 = uv[2] - uv[0], dv1 = uv[3] - uv[1], du2 = uv[4] - uv[0], dv2 = uv[5] - uv[1];
	float det = du1 * dv2 - du2 * dv1;
	if (!isfinite(det) || fabsf(det) < 1e-12f)
		return 0;
	pme_vec3 e1 = YMPME_VecSub(p[1], p[0]), e2 = YMPME_VecSub(p[2], p[0]);
	pme_vec3 t =
		YMPME_VecScale(YMPME_VecSub(YMPME_VecScale(e1, dv2), YMPME_VecScale(e2, dv1)), 1 / det);
	pme_vec3 b =
		YMPME_VecScale(YMPME_VecSub(YMPME_VecScale(e2, du1), YMPME_VecScale(e1, du2)), 1 / det);
	t = YMPME_VecSub(t, YMPME_VecScale(n, YMPME_VecDot(t, n)));
	float tl = length(t);
	if (!isfinite(tl) || tl < 1e-9f)
		return 0;
	t = YMPME_VecScale(t, 1 / tl);
	b = YMPME_VecSub(YMPME_VecSub(b, YMPME_VecScale(n, YMPME_VecDot(b, n))),
					 YMPME_VecScale(t, YMPME_VecDot(b, t)));
	float bl = length(b);
	if (!isfinite(bl) || bl < 1e-9f)
		return 0;
	b = YMPME_VecScale(b, 1 / bl);
	pme_vec3 result = V(YMPME_VecDot(f, t), YMPME_VecDot(f, b), -YMPME_VecDot(f, n));
	if (!YMPME_VecFinite(result))
		return 0;
	*out = result;
	return 1;
}
GRErgb24 DemoForce_RGB(pme_vec3 local, float normalRange, float tangentRange, int *clipped)
{
	if (!isfinite(normalRange) || normalRange <= 0 || !isfinite(tangentRange) || tangentRange <= 0)
	{
		if (clipped)
			*clipped = 1;
		return (GRErgb24){0, 128, 128};
	}
	int cap = !YMPME_VecFinite(local) || local.z < 0 || local.z > normalRange ||
			  fabsf(local.x) > tangentRange || fabsf(local.y) > tangentRange;
	if (clipped)
		*clipped = cap;
	if (!YMPME_VecFinite(local))
		return (GRErgb24){0, 128, 128};
	return (GRErgb24){(uint8)lroundf(255 * YMPME_Clamp(local.z / normalRange, 0, 1)),
					  (uint8)lroundf(127.5f * (1 + YMPME_Clamp(local.x / tangentRange, -1, 1))),
					  (uint8)lroundf(127.5f * (1 + YMPME_Clamp(local.y / tangentRange, -1, 1)))};
}
