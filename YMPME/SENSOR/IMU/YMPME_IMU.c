#include "YMPME_IMU.h"
static pme_vec3 noise3(uint32_t *seed)
{
	pme_vec3 v;
	v.x = YMPME_Sensor_UnitNoise(seed);
	v.y = YMPME_Sensor_UnitNoise(seed);
	v.z = YMPME_Sensor_UnitNoise(seed);
	return v;
}
pme_result YMPME_IMU_Measure(const pme_imu_desc *d, const pme_feedback_state *s,
							 const pme_sensor_stamp *stamp, uint32_t *seed, pme_imu_sample *out)
{
	unsigned required =
		PME_FEEDBACK_POSE | PME_FEEDBACK_ACCELERATION | PME_FEEDBACK_ANGULAR_VELOCITY;
	if (!d || !s || !stamp || !seed || !out || !stamp->valid || stamp->time != s->time ||
		!isfinite(stamp->time) || !isfinite(stamp->availableTime) ||
		stamp->availableTime < stamp->time || (s->valid & required) != required ||
		!YMPME_Rotation_IsValid(s->rotation) || !YMPME_Rotation_IsValid(d->mount.rotation) ||
		!YMPME_VecFinite(d->mount.offset) || !YMPME_VecFinite(d->gravity) ||
		!YMPME_VecFinite(d->accelBias) || !YMPME_VecFinite(d->gyroBias) ||
		!YMPME_VecFinite(s->acceleration) || !YMPME_VecFinite(s->angularVelocity) ||
		!isfinite(d->accelNoise) || d->accelNoise < 0 || !isfinite(d->gyroNoise) ||
		d->gyroNoise < 0)
		return PME_INVALID_ARGUMENT;
	pme_vec3 acc = s->acceleration;
	if (YMPME_VecDot(d->mount.offset, d->mount.offset) > 0)
	{
		if (!(s->valid & PME_FEEDBACK_ANGULAR_ACCELERATION) ||
			!YMPME_VecFinite(s->angularAcceleration))
			return PME_INVALID_ARGUMENT;
		pme_vec3 r = YMPME_Rotation_Apply(s->rotation, d->mount.offset);
		acc =
			YMPME_VecAdd(acc, YMPME_VecAdd(YMPME_VecCross(s->angularAcceleration, r),
										   YMPME_VecCross(s->angularVelocity,
														  YMPME_VecCross(s->angularVelocity, r))));
	}
	pme_rotation rot = YMPME_Sensor_WorldRotation(s->rotation, d->mount.rotation);
	pme_imu_sample result;
	result.stamp = *stamp;
	result.specificForce =
		YMPME_VecAdd(YMPME_Sensor_ToLocal(rot, YMPME_VecSub(acc, d->gravity)), d->accelBias);
	result.angularVelocity =
		YMPME_VecAdd(YMPME_Sensor_ToLocal(rot, s->angularVelocity), d->gyroBias);
	uint32_t rng = *seed;
	result.specificForce =
		YMPME_VecAdd(result.specificForce, YMPME_VecScale(noise3(&rng), d->accelNoise));
	result.angularVelocity =
		YMPME_VecAdd(result.angularVelocity, YMPME_VecScale(noise3(&rng), d->gyroNoise));
	if (!YMPME_VecFinite(result.specificForce) || !YMPME_VecFinite(result.angularVelocity))
		return PME_INVALID_ARGUMENT;
	*seed = rng;
	*out = result;
	return PME_OK;
}
