#include "YMPME_SkinContact.h"
#include "../../CORE/YMPME_MathBase.h"
pme_skin_contact_desc YMPME_SkinContact_DefaultDesc(void)
{
	return (pme_skin_contact_desc){200000, 100, .3f, 50, 5000};
}
static pme_result one(const pme_skin_contact_desc *d, const pme_surface_patch *p,
					  const pme_skin_probe *s, const pme_vec3 *omega, pme_vec3 *f, float *depth,
					  int *clipped)
{
	pme_vec3 delta = YMPME_VecSub(p->position, s->center);
	float distance2 = YMPME_VecDot(delta, delta);
	*f = YMPME_Vec3(0, 0, 0);
	*depth = 0;
	*clipped = 0;
	/* 球AABB粗筛选，再做采样点球面距离；背面不受吸引。 */
	if (fabsf(delta.x) >= s->radius || fabsf(delta.y) >= s->radius || fabsf(delta.z) >= s->radius ||
		distance2 >= s->radius * s->radius || YMPME_VecDot(delta, p->normal) >= 0)
		return PME_OK;
	float distance = sqrtf(distance2);
	*depth = s->radius - distance;
	pme_vec3 normal = p->normal;
	pme_vec3 relative = YMPME_VecSub(s->velocity, p->velocity);
	if (omega)
	{
		/* 动球使用径向法线，法向载荷不产生虚假自旋。 */
		if (distance > 1e-9f)
			normal = YMPME_VecScale(delta, -1 / distance);
		relative = YMPME_VecAdd(relative, YMPME_VecCross(*omega, delta));
	}
	float vn = YMPME_VecDot(relative, normal);
	double pressure = (double)d->stiffness * (*depth) - (double)d->damping * vn;
	*clipped = pressure > d->maxPressure;
	pressure = fmax(0, fmin(d->maxPressure, pressure));
	float magnitude = (float)(pressure * p->area);
	pme_vec3 tangent = YMPME_VecSub(relative, YMPME_VecScale(normal, vn));
	float speed = sqrtf(YMPME_VecDot(tangent, tangent));
	*f = YMPME_VecScale(normal, -magnitude);
	if (speed > 1e-9f)
	{
		float drag = (float)fmin((double)d->friction * magnitude,
								 (double)d->tangentDamping * p->area * speed);
		*f = YMPME_VecAdd(*f, YMPME_VecScale(tangent, drag / speed));
	}
	return YMPME_VecFinite(*f) ? PME_OK : PME_INVALID_ARGUMENT;
}
static pme_result sphere(const pme_skin_contact_desc *d, const pme_surface_patch *p, size_t n,
						 const pme_skin_probe *s, pme_vec3 *forces, size_t capacity,
						 pme_skin_contact_result *out, const pme_vec3 *omega)
{
	if ((omega && !YMPME_VecFinite(*omega)) || !d || !s || !out || (n && (!p || !forces)) ||
		!isfinite(d->stiffness) || d->stiffness < 0 || !isfinite(d->damping) || d->damping < 0 ||
		!isfinite(d->friction) || d->friction < 0 || !isfinite(d->tangentDamping) ||
		d->tangentDamping < 0 || !isfinite(d->maxPressure) || d->maxPressure <= 0 ||
		!YMPME_VecFinite(s->center) || !YMPME_VecFinite(s->velocity) || !isfinite(s->radius) ||
		s->radius <= 0 || !isfinite(s->radius * s->radius))
		return PME_INVALID_ARGUMENT;
	if (capacity < n)
		return PME_CAPACITY_EXCEEDED;
	pme_skin_contact_result result = {0};
	for (size_t i = 0; i < n; ++i)
	{
		pme_vec3 f;
		float depth;
		int clipped;
		if (!YMPME_VecFinite(p[i].position) || !YMPME_VecFinite(p[i].velocity) ||
			!YMPME_VecFinite(p[i].normal) ||
			fabsf(YMPME_VecDot(p[i].normal, p[i].normal) - 1) > 1e-4f || !isfinite(p[i].area) ||
			p[i].area <= 0 || one(d, &p[i], s, omega, &f, &depth, &clipped) != PME_OK)
			return PME_INVALID_ARGUMENT;
		result.probeWrench.force = YMPME_VecSub(result.probeWrench.force, f);
		result.probeWrench.torque = YMPME_VecSub(
			result.probeWrench.torque, YMPME_VecCross(YMPME_VecSub(p[i].position, s->center), f));
		if (YMPME_VecDot(f, f) > 0)
			++result.contacts;
		result.maxPenetration = fmaxf(result.maxPenetration, depth);
		result.saturated |= clipped;
	}
	if (!YMPME_VecFinite(result.probeWrench.force) || !YMPME_VecFinite(result.probeWrench.torque))
		return PME_INVALID_ARGUMENT;
	for (size_t i = 0; i < n; ++i)
	{
		float depth;
		int clipped;
		one(d, &p[i], s, omega, &forces[i], &depth, &clipped);
	}
	*out = result;
	return PME_OK;
}

pme_result YMPME_SkinContact_Sphere(const pme_skin_contact_desc *d, const pme_surface_patch *p,
									size_t n, const pme_skin_probe *s, pme_vec3 *f, size_t capacity,
									pme_skin_contact_result *out)
{
	return sphere(d, p, n, s, f, capacity, out, NULL);
}
pme_result YMPME_SkinContact_SphereRigid(const pme_skin_contact_desc *d, const pme_surface_patch *p,
										 size_t n, const pme_skin_probe *s, pme_vec3 omega,
										 pme_vec3 *f, size_t capacity, pme_skin_contact_result *out)
{
	return sphere(d, p, n, s, f, capacity, out, &omega);
}
