#include "../../CORE/YMPME_MathBase.h"
#include "YMPME_SkinPlane.h"
static int one(const pme_skin_contact_desc *d, const pme_surface_patch *p,
			   const pme_skin_plane *plane, pme_vec3 *force, float *depth, int *clipped)
{
	*force = YMPME_Vec3(0, 0, 0);
	*depth = 0;
	*clipped = 0;
	float facing = -YMPME_VecDot(p->normal, plane->normal);
	if (facing <= 0)
		return 1;
	double gap = (double)YMPME_VecDot(p->position, plane->normal) - plane->offset;
	if (gap >= plane->shell)
		return 1;
	*depth = (float)(plane->shell - gap);
	pme_vec3 relative = YMPME_VecSub(p->velocity, plane->velocity);
	float vn = YMPME_VecDot(relative, plane->normal);
	double pressure = (double)d->stiffness * *depth - (double)d->damping * vn;
	*clipped = pressure > d->maxPressure;
	float load = (float)(fmax(0, fmin(d->maxPressure, pressure)) * p->area * facing);
	*force = YMPME_VecScale(plane->normal, load);
	pme_vec3 tangent = YMPME_VecSub(relative, YMPME_VecScale(plane->normal, vn));
	float speed = sqrtf(YMPME_VecDot(tangent, tangent));
	if (speed > 1e-9f)
	{
		float drag = (float)fmin((double)d->friction * load,
								 (double)d->tangentDamping * p->area * facing * speed);
		*force = YMPME_VecSub(*force, YMPME_VecScale(tangent, drag / speed));
	}
	return YMPME_VecFinite(*force) && isfinite(*depth);
}
pme_result YMPME_SkinContact_Plane(const pme_skin_contact_desc *d, const pme_surface_patch *p,
								   size_t n, const pme_skin_plane *plane, pme_vec3 *forces,
								   size_t capacity, pme_skin_contact_result *out)
{
	if (!d || !plane || !out || (n && (!p || !forces)) || !isfinite(d->stiffness) ||
		d->stiffness < 0 || !isfinite(d->damping) || d->damping < 0 || !isfinite(d->friction) ||
		d->friction < 0 || !isfinite(d->tangentDamping) || d->tangentDamping < 0 ||
		!isfinite(d->maxPressure) || d->maxPressure <= 0 || !YMPME_VecFinite(plane->normal) ||
		fabsf(YMPME_VecDot(plane->normal, plane->normal) - 1) > 1e-4f ||
		!YMPME_VecFinite(plane->velocity) || !YMPME_VecFinite(plane->reference) ||
		!isfinite(plane->offset) || !isfinite(plane->shell) || plane->shell < 0)
		return PME_INVALID_ARGUMENT;
	if (capacity < n)
		return PME_CAPACITY_EXCEEDED;
	pme_skin_contact_result result = {0};
	for (size_t i = 0; i < n; i++)
	{
		pme_vec3 f;
		float depth;
		int clipped;
		if (!YMPME_VecFinite(p[i].position) || !YMPME_VecFinite(p[i].velocity) ||
			!YMPME_VecFinite(p[i].normal) ||
			fabsf(YMPME_VecDot(p[i].normal, p[i].normal) - 1) > 1e-4f || !isfinite(p[i].area) ||
			p[i].area <= 0 || !one(d, p + i, plane, &f, &depth, &clipped))
			return PME_INVALID_ARGUMENT;
		result.probeWrench.force = YMPME_VecSub(result.probeWrench.force, f);
		result.probeWrench.torque =
			YMPME_VecSub(result.probeWrench.torque,
						 YMPME_VecCross(YMPME_VecSub(p[i].position, plane->reference), f));
		result.contacts += YMPME_VecDot(f, f) > 0;
		result.maxPenetration = fmaxf(result.maxPenetration, depth);
		result.saturated |= clipped;
	}
	if (!YMPME_VecFinite(result.probeWrench.force) || !YMPME_VecFinite(result.probeWrench.torque))
		return PME_INVALID_ARGUMENT;
	for (size_t i = 0; i < n; i++)
	{
		float depth;
		int clipped;
		one(d, p + i, plane, forces + i, &depth, &clipped);
	}
	*out = result;
	return PME_OK;
}
