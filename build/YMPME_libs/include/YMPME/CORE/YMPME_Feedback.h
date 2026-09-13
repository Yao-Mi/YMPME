#ifndef YMPME_FEEDBACK_H
#define YMPME_FEEDBACK_H
#include "YMPME_ConvexMesh.h"
/* 公共真值格式，不依赖SENSOR或GRE；所有向量均为世界坐标，SI单位。
 * rotation:本体->世界。acceleration是interval内速度差分均值，包含接触/阻尼；
 * 初始瞬间尚无导数时清除对应有效位。时间由仿真调用方统一，禁止用渲染帧时间。 */
enum
{
	PME_FEEDBACK_POSE = 1,
	PME_FEEDBACK_VELOCITY = 2,
	PME_FEEDBACK_ACCELERATION = 4,
	PME_FEEDBACK_ANGULAR_VELOCITY = 8,
	PME_FEEDBACK_ANGULAR_ACCELERATION = 16
};
typedef struct pme_feedback_state_
{
	double time;
	float interval;
	unsigned valid;
	pme_vec3 position, velocity, acceleration, angularVelocity, angularAcceleration;
	pme_rotation rotation;
} pme_feedback_state;
typedef struct pme_wrench_
{
	pme_vec3 force, torque;
} pme_wrench;
/* 每项为一次求解迭代施加的增量冲量，均作用于B；A受反向冲量。
 * point 是AABB重叠区域中心估计，不是接触流形。不可视作真实皮肤压力分布。
 * slot索引只在本次步进内有效；sample消费完前不得删除/复用Body槽位。 */
typedef struct pme_contact_feedback_
{
	size_t bodyA, bodyB;
	unsigned iteration;
	pme_vec3 point, normal, normalImpulse, tangentImpulse;
	int approximatePoint;
} pme_contact_feedback;
/* 调用方提供缓冲，状态容量>=World.capacity；contacts可为空且capacity=0。
 * count为实际写入项数；dropped为被截断的项数，不影响物理。
 * 时间time由调用方传入步末仿真时间。所有缓冲不得互相/与World重叠。 */
typedef struct pme_world_feedback_
{
	pme_feedback_state *states;
	size_t stateCapacity;
	pme_contact_feedback *contacts;
	size_t contactCapacity, count, dropped;
	double time;
	float interval;
	size_t stateCount; /* 实际World容量，stateCount以外不是有效样本。 */
} pme_world_feedback;
#endif
