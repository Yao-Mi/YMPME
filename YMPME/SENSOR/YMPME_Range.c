#include "YMPME_RangeInternal.h"
int YMPME_Range_Unit(pme_vec3 v)
{
	return YMPME_VecFinite(v) && fabsf(YMPME_VecDot(v, v) - 1) <= 1e-4f;
}
int YMPME_Range_Pose(const pme_feedback_state *s, const pme_sensor_mount *m,
					 const pme_sensor_stamp *t, pme_vec3 *p, pme_rotation *r)
{
	if (!s || !m || !t || !t->valid || !isfinite(t->time) || !isfinite(t->availableTime) ||
		t->availableTime < t->time || t->time != s->time || !(s->valid & PME_FEEDBACK_POSE) ||
		!YMPME_VecFinite(s->position) || !YMPME_VecFinite(m->offset) ||
		!YMPME_Rotation_IsValid(s->rotation) || !YMPME_Rotation_IsValid(m->rotation))
		return 0;
	*p = YMPME_VecAdd(s->position, YMPME_Rotation_Apply(s->rotation, m->offset));
	*r = YMPME_Sensor_WorldRotation(s->rotation, m->rotation);
	return YMPME_VecFinite(*p);
}
pme_result YMPME_Range_Cast(const pme_ray_provider *provider, pme_vec3 origin,
							pme_rotation rotation, pme_vec3 direction, float max, pme_ray_hit *hit)
{
	pme_ray ray = {origin, YMPME_Rotation_Apply(rotation, direction), max};
	pme_result status;
	*hit = (pme_ray_hit){0, 0, {0, 0, 0}, {0, 0, 0}, SIZE_MAX, SIZE_MAX};
	status = provider->cast(provider->context, &ray, hit);
	if (status != PME_OK)
		return status;
	if (hit->hit && (!isfinite(hit->distance) || hit->distance < 0 || hit->distance > max))
		return PME_INVALID_ARGUMENT;
	return PME_OK;
}
