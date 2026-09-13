#include "YMPME_Sensor.h"
#include <string.h>
pme_result YMPME_SensorClock_Init(pme_sensor_clock *c, double period, double latency)
{
	if (!c || !isfinite(period) || period <= 0 || !isfinite(latency) || latency < 0)
		return PME_INVALID_ARGUMENT;
	memset(c, 0, sizeof(*c));
	c->period = period;
	c->latency = latency;
	return PME_OK;
}
int YMPME_SensorClock_Due(const pme_sensor_clock *c, double t)
{
	return c && isfinite(t) && t >= 0 && isfinite(c->period) && c->period > 0 &&
		   (!c->started || (t >= c->lastTime && t + fmin(1e-9, c->period * 1e-6) >= c->nextTime));
}
pme_result YMPME_SensorClock_Stamp(pme_sensor_clock *c, double t, pme_sensor_stamp *out)
{
	if (!out || !YMPME_SensorClock_Due(c, t) || !isfinite(c->latency) || c->latency < 0 ||
		!isfinite(t + c->latency))
		return PME_INVALID_ARGUMENT;
	double count =
		c->started
			? floor(fmax(0, (t - c->nextTime + fmin(1e-9, c->period * 1e-6)) / c->period)) + 1
			: 1;
	if (count >= UINT64_MAX || count > (double)(UINT64_MAX - c->missed) ||
		c->sequence == UINT64_MAX)
		return PME_INVALID_ARGUMENT;
	double next = c->started ? c->nextTime + count * c->period : t + c->period;
	if (!isfinite(next) || next <= t)
		return PME_INVALID_ARGUMENT;
	c->missed += (uint64_t)count - 1;
	c->nextTime = next;
	c->lastTime = t;
	c->started = 1;
	*out = (pme_sensor_stamp){t, t + c->latency, ++c->sequence, 1};
	return PME_OK;
}
pme_sensor_mount YMPME_Sensor_IdentityMount(void)
{
	pme_sensor_mount m = {YMPME_Rotation_Identity(), {0, 0, 0}};
	return m;
}
pme_rotation YMPME_Sensor_WorldRotation(pme_rotation a, pme_rotation b)
{
	pme_rotation r;
	for (int i = 0; i < 3; ++i)
		r.axis[i] = YMPME_Rotation_Apply(a, b.axis[i]);
	return r;
}
pme_vec3 YMPME_Sensor_ToLocal(pme_rotation r, pme_vec3 v)
{
	return YMPME_Vec3(YMPME_VecDot(r.axis[0], v), YMPME_VecDot(r.axis[1], v),
					  YMPME_VecDot(r.axis[2], v));
}
float YMPME_Sensor_UnitNoise(uint32_t *seed)
{
	uint32_t n = *seed ? *seed : UINT32_C(0x42c0ffee);
	n ^= n << 13;
	n ^= n >> 17;
	n ^= n << 5;
	*seed = n;
	return ((n >> 8) * (1.0f / 16777216) * 2 - 1) * 1.73205080757f;
}
