#include "YMPME_Laser.h"
#include "../YMPME_RangeInternal.h"
pme_laser_desc YMPME_Laser_DefaultDesc(void)
{
	pme_laser_desc d = {YMPME_Sensor_IdentityMount(), .02f, 20, 0, 0, .001f};
	return d;
}
pme_result YMPME_Laser_Measure(const pme_laser_desc *d, const pme_feedback_state *state,
							   const pme_sensor_stamp *stamp, const pme_ray_provider *provider,
							   uint32_t *seed, pme_laser_sample *out)
{
	pme_laser_sample s = {0};
	pme_rotation rotation;
	pme_ray_hit hit;
	if (!d || !out || !provider || !provider->cast || !seed || !isfinite(d->minRange) ||
		d->minRange < 0 || !isfinite(d->maxRange) || d->maxRange <= d->minRange ||
		!isfinite(d->bias) || !isfinite(d->noise) || d->noise < 0 || !isfinite(d->resolution) ||
		d->resolution < 0 || !YMPME_Range_Pose(state, &d->mount, stamp, &s.origin, &rotation))
		return PME_INVALID_ARGUMENT;
	s.direction = rotation.axis[2];
	pme_result result =
		YMPME_Range_Cast(provider, s.origin, rotation, YMPME_Vec3(0, 0, 1), d->maxRange, &hit);
	if (result != PME_OK)
		return result;
	s.stamp = *stamp;
	s.distance = NAN;
	s.status = PME_LASER_NO_RETURN;
	s.hit = !!hit.hit;
	s.body = s.hit ? hit.body : SIZE_MAX;
	s.rayEnd =
		YMPME_VecAdd(s.origin, YMPME_VecScale(s.direction, s.hit ? hit.distance : d->maxRange));
	if (!YMPME_VecFinite(s.rayEnd))
		return PME_INVALID_ARGUMENT;
	uint32_t rng = *seed;
	if (s.hit)
	{
		s.status = PME_LASER_BLIND;
		if (hit.distance >= d->minRange)
		{
			double value = (double)hit.distance + d->bias;
			if (d->noise > 0)
				value += (double)d->noise * YMPME_Sensor_UnitNoise(&rng);
			if (d->resolution > 0)
				value = round(value / d->resolution) * d->resolution;
			s.status = PME_LASER_OUT_OF_RANGE;
			if (isfinite(value) && value >= d->minRange && value <= d->maxRange)
			{
				s.valid = 1;
				s.status = PME_LASER_VALID;
				s.distance = (float)value;
			}
		}
	}
	*out = s;
	*seed = rng;
	return PME_OK;
}
