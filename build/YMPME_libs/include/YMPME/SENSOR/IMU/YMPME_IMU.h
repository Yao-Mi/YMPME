#ifndef YMPME_IMU_H
#define YMPME_IMU_H
#include "../YMPME_Sensor.h"
#ifdef __cplusplus
extern "C"
{
#endif
	typedef struct pme_imu_desc_
	{
		pme_sensor_mount mount;
		pme_vec3 gravity, accelBias, gyroBias;
		float accelNoise, gyroNoise;
	} pme_imu_desc;
	typedef struct pme_imu_sample_
	{
		pme_sensor_stamp stamp;
		pme_vec3 specificForce, angularVelocity;
	} pme_imu_sample;
	/* 安装坐标下比力(m/s²)和角速度(rad/s)。不输出虚构的姿态观测。
	 * 包含安装偏移的alpha×r及omega×(omega×r)，噪声幅度单位同输出。
	 * 失败不改输出/种子；stamp.time必须匹配真值时间。 */
	pme_result YMPME_IMU_Measure(const pme_imu_desc *desc, const pme_feedback_state *state,
								 const pme_sensor_stamp *stamp, uint32_t *seed,
								 pme_imu_sample *out);
#ifdef __cplusplus
}
#endif
#endif
