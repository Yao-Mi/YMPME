#ifndef YMPME_SENSOR_H
#define YMPME_SENSOR_H
#include "../CORE/YMPME_Feedback.h"
#include "../CORE/YMPME_MathBase.h"
#ifdef __cplusplus
extern "C"
{
#endif
	/* 时钟与数据格式独立：未来触觉数组/点云/深度图保留各自载荷类型。
	 * 本层不调度物理。应用按Due决定采样，按availableTime排队交付，不得提前消费延迟样本。
	 * 不补采丢失的历史状态；跳过的采样点记入missed。每实例独立、无堆分配。 */
	typedef struct pme_sensor_stamp_
	{
		double time, availableTime;
		uint64_t sequence;
		unsigned valid;
	} pme_sensor_stamp;
	typedef struct pme_sensor_clock_
	{
		double period, latency, nextTime, lastTime;
		uint64_t sequence, missed;
		int started;
	} pme_sensor_clock;
	pme_result YMPME_SensorClock_Init(pme_sensor_clock *clock, double period, double latency);
	int YMPME_SensorClock_Due(const pme_sensor_clock *clock, double time);
	pme_result YMPME_SensorClock_Stamp(pme_sensor_clock *clock, double time, pme_sensor_stamp *out);
	/* 坐标安装：rotation为传感器->本体，offset为质心到传感器(m，本体坐标)。 */
	typedef struct pme_sensor_mount_
	{
		pme_rotation rotation;
		pme_vec3 offset;
	} pme_sensor_mount;
	pme_sensor_mount YMPME_Sensor_IdentityMount(void);
	pme_rotation YMPME_Sensor_WorldRotation(pme_rotation body, pme_rotation mount);
	pme_vec3 YMPME_Sensor_ToLocal(pme_rotation rotation, pme_vec3 world);
	/* 使用调用方持有的种子产生[-sqrt(3),sqrt(3)]均匀噪声，均值0方差1；非高斯。 */
	float YMPME_Sensor_UnitNoise(uint32_t *seed);
#ifdef __cplusplus
}
#endif
#endif
