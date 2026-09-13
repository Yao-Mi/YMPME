#include "YMPME_SurfaceTactile.h"
static int cell(const pme_surface_patch *p, pme_vec3 f, pme_surface_taxel *c)
{
	if (!YMPME_VecFinite(p->position) || !YMPME_VecFinite(p->normal) ||
		fabsf(YMPME_VecDot(p->normal, p->normal) - 1) > 1e-4f || !isfinite(p->area) ||
		p->area <= 0 || !YMPME_VecFinite(f))
		return 0;
	float signedForce = YMPME_VecDot(f, p->normal);
	c->force = f;
	c->normalForce = fmaxf(0, -signedForce);
	c->pressure = c->normalForce / p->area;
	c->shear = YMPME_VecSub(f, YMPME_VecScale(p->normal, signedForce));
	return isfinite(c->pressure) && YMPME_VecFinite(c->shear);
}
pme_result YMPME_SurfaceTactile_Measure(const pme_surface_patch *p, const pme_vec3 *f, size_t n,
										const pme_sensor_stamp *stamp, pme_vec3 origin,
										int approximate, pme_surface_tactile_frame *out)
{
	if (!out || !stamp || !stamp->valid || !isfinite(stamp->time) || stamp->time < 0 ||
		!isfinite(stamp->availableTime) || stamp->availableTime < stamp->time ||
		!YMPME_VecFinite(origin) || (n && (!p || !f || !out->cells)))
		return PME_INVALID_ARGUMENT;
	if (out->capacity < n)
		return PME_CAPACITY_EXCEEDED;
	pme_surface_tactile_frame frame = *out;
	frame.wrench = (pme_wrench){0};
	for (size_t i = 0; i < n; ++i)
	{
		pme_surface_taxel c;
		if (!cell(&p[i], f[i], &c))
			return PME_INVALID_ARGUMENT;
		frame.wrench.force = YMPME_VecAdd(frame.wrench.force, f[i]);
		frame.wrench.torque = YMPME_VecAdd(
			frame.wrench.torque, YMPME_VecCross(YMPME_VecSub(p[i].position, origin), f[i]));
	}
	if (!YMPME_VecFinite(frame.wrench.force) || !YMPME_VecFinite(frame.wrench.torque))
		return PME_INVALID_ARGUMENT;
	for (size_t i = 0; i < n; ++i)
		cell(&p[i], f[i], &frame.cells[i]);
	frame.stamp = *stamp;
	frame.count = n;
	frame.origin = origin;
	frame.approximate = approximate;
	*out = frame;
	return PME_OK;
}
