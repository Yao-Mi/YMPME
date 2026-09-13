#ifndef YMPME_IMU_DEVICE_H
#define YMPME_IMU_DEVICE_H
#include "YMPME_IMU.h"
#include "../YMPME_Signal.h"
#ifdef __cplusplus
extern "C"
{
#endif
	typedef struct pme_imu_device_
	{
		pme_imu_desc desc;
		pme_signal_desc accelerometer, gyroscope;
		pme_signal_state channels[6];
		uint32_t seed;
	} pme_imu_device;
	typedef struct pme_imu_reading_
	{
		pme_imu_sample sample;
		unsigned saturated;
	} pme_imu_reading; /* bits0..2加速度，3..5角速度。 */
	pme_result YMPME_IMUDevice_Init(pme_imu_device *device, const pme_imu_desc *desc,
									const pme_signal_desc *accelerometer,
									const pme_signal_desc *gyroscope, uint32_t seed);
	/* 原有IMU负责安装与比力/噪声，此层增加量程、校准、ADC量化、时间低通。
	 * 时间必须递增；失败不改设备/种子/输出。不输出姿态估计。 */
	pme_result YMPME_IMUDevice_Measure(pme_imu_device *device, const pme_feedback_state *state,
									   const pme_sensor_stamp *stamp, pme_imu_reading *out);
#ifdef __cplusplus
}
#endif
#endif
