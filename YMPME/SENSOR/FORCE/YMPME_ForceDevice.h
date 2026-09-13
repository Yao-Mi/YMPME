#ifndef YMPME_FORCE_DEVICE_H
#define YMPME_FORCE_DEVICE_H
#include "YMPME_ForceSensor.h"
#include "../YMPME_Signal.h"
#ifdef __cplusplus
extern "C"
{
#endif
	typedef struct pme_force_device_desc_
	{
		pme_sensor_mount mount;
		pme_wrench bias;
		float forceNoise, torqueNoise;
		pme_signal_desc force, torque;
	} pme_force_device_desc;
	typedef struct pme_force_device_
	{
		pme_force_device_desc desc;
		pme_wrench tare;
		pme_signal_state channels[6];
		uint32_t seed;
		int tarePending;
	} pme_force_device;
	typedef struct pme_force_reading_
	{
		pme_force_sample sample;
		unsigned saturated;
	} pme_force_reading;
	pme_force_device_desc YMPME_ForceDevice_DefaultDesc(void);
	pme_result YMPME_ForceDevice_Init(pme_force_device *device, const pme_force_device_desc *desc,
									  uint32_t seed);
	/* 下一次成功测量捕获零点，含当前载荷/偏置/噪声；同时重置滤波器。
	 * 去皮后读数为相对载荷。重新Init恢复绝对载荷，不改物理接触力。 */
	void YMPME_ForceDevice_TareNext(pme_force_device *device);
	/* input.wrench为世界坐标、关于state质心的载荷；保留approximateTorque。
	 * 先转换安装坐标，再加偏置/均匀噪声、减零点、滤波/限幅/量化。
	 * 失败不提交设备状态、去皮请求、随机种子和输出。 */
	pme_result YMPME_ForceDevice_Measure(pme_force_device *device, const pme_force_sample *input,
										 const pme_feedback_state *state, pme_force_reading *out);
#ifdef __cplusplus
}
#endif
#endif
