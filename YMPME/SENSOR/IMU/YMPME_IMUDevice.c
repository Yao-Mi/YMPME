#include "YMPME_IMUDevice.h"
#include <string.h>
pme_result YMPME_IMUDevice_Init(pme_imu_device *d, const pme_imu_desc *desc,
								const pme_signal_desc *accel, const pme_signal_desc *gyro,
								uint32_t seed)
{
	uint32_t checkSeed = seed;
	pme_imu_device v;
	pme_feedback_state s = {0};
	pme_sensor_stamp t = {0, 0, 0, 1};
	pme_imu_sample sample;
	if (!d || !desc || !YMPME_Signal_DescValid(accel) || !YMPME_Signal_DescValid(gyro))
		return PME_INVALID_ARGUMENT;
	s.valid = 31;
	s.rotation = YMPME_Rotation_Identity();
	if (YMPME_IMU_Measure(desc, &s, &t, &checkSeed, &sample) != PME_OK)
		return PME_INVALID_ARGUMENT;
	memset(&v, 0, sizeof(v));
	v.desc = *desc;
	v.accelerometer = *accel;
	v.gyroscope = *gyro;
	v.seed = seed;
	/* Init validation must not consume the user's RNG sequence. */
	*d = v;
	return PME_OK;
}
pme_result YMPME_IMUDevice_Measure(pme_imu_device *d, const pme_feedback_state *s,
								   const pme_sensor_stamp *stamp, pme_imu_reading *out)
{
	pme_imu_device next;
	pme_imu_reading value = {0};
	float input[6], output[6];
	int i, clipped;
	if (!d || !out)
		return PME_INVALID_ARGUMENT;
	next = *d;
	if (YMPME_IMU_Measure(&next.desc, s, stamp, &next.seed, &value.sample) != PME_OK)
		return PME_INVALID_ARGUMENT;
	input[0] = value.sample.specificForce.x;
	input[1] = value.sample.specificForce.y;
	input[2] = value.sample.specificForce.z;
	input[3] = value.sample.angularVelocity.x;
	input[4] = value.sample.angularVelocity.y;
	input[5] = value.sample.angularVelocity.z;
	for (i = 0; i < 6; ++i)
	{
		if (YMPME_Signal_Process(i < 3 ? &next.accelerometer : &next.gyroscope, &next.channels[i],
								 input[i], stamp->time, &output[i], &clipped) != PME_OK)
			return PME_INVALID_ARGUMENT;
		if (clipped)
			value.saturated |= 1u << i;
	}
	value.sample.specificForce = YMPME_Vec3(output[0], output[1], output[2]);
	value.sample.angularVelocity = YMPME_Vec3(output[3], output[4], output[5]);
	*d = next;
	*out = value;
	return PME_OK;
}
