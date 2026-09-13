#ifndef YMPME_TACTILE_H
#define YMPME_TACTILE_H
#include "../YMPME_Sensor.h"
#ifdef __cplusplus
extern "C"
{
#endif
	typedef struct pme_tactile_desc_
	{
		pme_sensor_mount mount;
		size_t width, height;
		float cellWidth, cellHeight, planeTolerance; /* m；平面以安装点为中心，本地XY，+Z朝外。 */
	} pme_tactile_desc;
	typedef struct pme_tactile_contact_
	{
		pme_vec3 point, force;
		int approximate;
	} pme_tactile_contact; /* 世界坐标、作用于皮肤的力N。 */
	typedef struct pme_taxel_
	{
		pme_vec3 force; /* 安装坐标；x/y为剪切力，-z为压缩力。 */
		float pressure; /* max(0,-force.z)/(cellWidth*cellHeight)，单位Pa。 */
		size_t contributions;
	} pme_taxel;
	typedef struct pme_tactile_frame_
	{
		pme_sensor_stamp stamp;
		pme_vec3 origin;
		pme_rotation rotation;
		pme_taxel *cells;
		size_t capacity, width, height, accepted, rejected;
		pme_vec3 totalForce;
		int approximate; /* 已接收载荷之和，安装坐标。 */
	} pme_tactile_frame;
	/* 输入为已求解的接触力或外部接触模型载荷，不从位姿猜测压力。
	 * 以最近网格单元归集；外边界半开区间，离平面过远/拉力/网格外会计入rejected。
	 * 不模拟材料弹性、接触面积扩散、皮肤形变；pressure是单元面积均值。
	 * 失败前置校验不改输出；计算溢出会使stamp.valid=0，cells可能已写入。 */
	pme_result YMPME_Tactile_Measure(const pme_tactile_desc *desc, const pme_feedback_state *state,
									 const pme_sensor_stamp *stamp,
									 const pme_tactile_contact *contacts, size_t count,
									 pme_tactile_frame *out);
	/* 旧World所有迭代增量冲量/dt直接归集，不丢迭代项；拒绝反馈缓冲截断。
	 * approximatePoint会传递给frame.approximate。是单个物理步平均力，不能抽取
	 * 低频帧冒充整个采样窗口平均力。可用Measure接外部精确接触流形。 */
	pme_result YMPME_Tactile_FromContacts(const pme_tactile_desc *desc,
										  const pme_feedback_state *state,
										  const pme_sensor_stamp *stamp,
										  const pme_world_feedback *feedback, size_t skinBody,
										  pme_tactile_frame *out);
#ifdef __cplusplus
}
#endif
#endif
