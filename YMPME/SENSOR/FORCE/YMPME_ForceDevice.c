#include "YMPME_ForceDevice.h"
#include <string.h>
pme_force_device_desc YMPME_ForceDevice_DefaultDesc(void)
{
	pme_force_device_desc d = {0};
	d.mount = YMPME_Sensor_IdentityMount();
	d.force = d.torque = YMPME_Signal_DefaultDesc();
	return d;
}
pme_result YMPME_ForceDevice_Init(pme_force_device *s, const pme_force_device_desc *d,
								  uint32_t seed)
{
	pme_force_device v;
	if (!s || !d || !YMPME_Signal_DescValid(&d->force) || !YMPME_Signal_DescValid(&d->torque) ||
		!YMPME_Rotation_IsValid(d->mount.rotation) || !YMPME_VecFinite(d->mount.offset) ||
		!YMPME_VecFinite(d->bias.force) || !YMPME_VecFinite(d->bias.torque) ||
		!isfinite(d->forceNoise) || d->forceNoise < 0 || !isfinite(d->torqueNoise) ||
		d->torqueNoise < 0)
		return PME_INVALID_ARGUMENT;
	memset(&v, 0, sizeof(v));
	v.desc = *d;
	v.seed = seed;
	*s = v;
	return PME_OK;
}
void YMPME_ForceDevice_TareNext(pme_force_device *d)
{
	if (d)
		d->tarePending = 1;
}
pme_result YMPME_ForceDevice_Measure(pme_force_device *d, const pme_force_sample *input,
									 const pme_feedback_state *s, pme_force_reading *out)
{
	pme_force_device next;
	pme_force_reading reading = {0};
	float raw[6], bias[6], tare[6], result[6];
	int i, clipped;
	if (!d || !input || !out || !isfinite(d->desc.forceNoise) || d->desc.forceNoise < 0 ||
		!isfinite(d->desc.torqueNoise) || d->desc.torqueNoise < 0)
		return PME_INVALID_ARGUMENT;
	next = *d;
	/* Validate monotonic capture time even when a tare resets the filters. */
	if (next.channels[0].initialized && input->stamp.time <= next.channels[0].time)
		return PME_INVALID_ARGUMENT;
	if (YMPME_ForceSensor_Measure(&input->wrench, s, &next.desc.mount, &input->stamp,
								  &reading.sample) != PME_OK)
		return PME_INVALID_ARGUMENT;
	raw[0] = reading.sample.wrench.force.x;
	raw[1] = reading.sample.wrench.force.y;
	raw[2] = reading.sample.wrench.force.z;
	raw[3] = reading.sample.wrench.torque.x;
	raw[4] = reading.sample.wrench.torque.y;
	raw[5] = reading.sample.wrench.torque.z;
	bias[0] = next.desc.bias.force.x;
	bias[1] = next.desc.bias.force.y;
	bias[2] = next.desc.bias.force.z;
	bias[3] = next.desc.bias.torque.x;
	bias[4] = next.desc.bias.torque.y;
	bias[5] = next.desc.bias.torque.z;
	tare[0] = next.tare.force.x;
	tare[1] = next.tare.force.y;
	tare[2] = next.tare.force.z;
	tare[3] = next.tare.torque.x;
	tare[4] = next.tare.torque.y;
	tare[5] = next.tare.torque.z;
	for (i = 0; i < 6; ++i)
	{
		float noise = i < 3 ? next.desc.forceNoise : next.desc.torqueNoise;
		raw[i] += bias[i];
		if (noise > 0)
			raw[i] += noise * YMPME_Sensor_UnitNoise(&next.seed);
		if (!isfinite(raw[i]))
			return PME_INVALID_ARGUMENT;
		if (next.tarePending)
		{
			tare[i] = raw[i];
			next.channels[i] = (pme_signal_state){0};
		}
		if (YMPME_Signal_Process(i < 3 ? &next.desc.force : &next.desc.torque, &next.channels[i],
								 raw[i] - tare[i], input->stamp.time, &result[i],
								 &clipped) != PME_OK)
			return PME_INVALID_ARGUMENT;
		if (clipped)
			reading.saturated |= 1u << i;
	}
	next.tare.force = YMPME_Vec3(tare[0], tare[1], tare[2]);
	next.tare.torque = YMPME_Vec3(tare[3], tare[4], tare[5]);
	next.tarePending = 0;
	reading.sample.wrench.force = YMPME_Vec3(result[0], result[1], result[2]);
	reading.sample.wrench.torque = YMPME_Vec3(result[3], result[4], result[5]);
	reading.sample.approximateTorque = input->approximateTorque;
	*d = next;
	*out = reading;
	return PME_OK;
}
