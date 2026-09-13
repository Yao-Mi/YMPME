#ifndef YMPME_FORCESENSOR_H
#define YMPME_FORCESENSOR_H
#include "../YMPME_Sensor.h"
#ifdef __cplusplus
extern "C"
{
#endif
	typedef struct pme_force_sample_
	{
		pme_sensor_stamp stamp;
		pme_wrench wrench;
		int approximateTorque;
	} pme_force_sample;
	/* 汇总本次步进所有迭代增量冲量/dt；丢项时拒绝输出，避免静默低估载荷。
	 * 输出世界坐标系、关于origin的合力/力矩。单刚体接触测力，不含外加力/重力。
	 * approximateTorque=1表示接触位置为估计，不能当关节精确扭矩。 */
	pme_result YMPME_ForceSensor_Contacts(const pme_world_feedback *feedback, size_t body,
										  pme_vec3 origin, pme_force_sample *out);
	/* 把关于质心的世界wrench移到传感器安装点，并转到安装坐标系。 */
	pme_result YMPME_ForceSensor_Measure(const pme_wrench *wrench, const pme_feedback_state *state,
										 const pme_sensor_mount *mount,
										 const pme_sensor_stamp *stamp, pme_force_sample *out);
	/* 低频测力窗口：连续累积每个物理步的冲量，Read除以整个窗口时间。
	 * origin固定世界坐标；窗口内不可删除/复用目标Body。Read不清空，Begin显式重置。
	 * 漏步、重复步、接触缓冲溢出均拒绝累积，不修改窗口。 */
	typedef struct pme_force_window_
	{
		pme_wrench impulse;
		pme_vec3 origin;
		size_t body;
		double startTime, endTime, duration;
		int active, approximateTorque;
	} pme_force_window;
	pme_result YMPME_ForceWindow_Begin(pme_force_window *window, size_t body, pme_vec3 origin);
	pme_result YMPME_ForceWindow_Add(pme_force_window *window, const pme_world_feedback *feedback);
	pme_result YMPME_ForceWindow_Read(const pme_force_window *window, pme_force_sample *out);
#ifdef __cplusplus
}
#endif
#endif
