/* 固定双轴皮带预设，96 个节点。Init 设置默认尺寸、质量与零速度。
 * collision=0 理想约束，1 接触摩擦；初始布局/工作区由 Init 建立。
 * p/v/contactWheel 为同一实例的连续状态，可直接只读绑定；不应浅拷贝成共享状态。
 * target(rad/s)、motor、mu(>=0)可在步间改变；几何/质量/工作缓存保持初始化配置。
 * Step 接受 0<dt<=0.01 秒，建议 dt<=1/600；非法 dt/空指针不修改状态。
 * 本模型不含绳身自碰撞；固定内置缓冲，无分配，未调用 ROPE 的无碰撞求解器。 */
#ifndef YMPME_BELT_H
#define YMPME_BELT_H
#include "../../CORE/YMPME_MathBase.h"
#define PME_BELT_PI 3.14159265358979323846
#define PME_BELT_N 96
#ifdef __cplusplus
extern "C"
{
#endif
	typedef struct pme_belt_point_
	{
		double x, y;
	} pme_belt_point;
	typedef struct pme_belt_
	{
		int collision, motor, contacts;
		double angle[2], omega[2], target, mu, radius[2], inertia[2], distance, nodeRadius;
		double rest[PME_BELT_N], lambda[PME_BELT_N], normalImpulse[PME_BELT_N], contactTorque, phase,
			maxStretch;
		pme_belt_point p[PME_BELT_N], v[PME_BELT_N], old[PME_BELT_N];
		int contactWheel[PME_BELT_N];
	} pme_belt;
	void YMPME_Belt_Init(pme_belt *b, int collision);
	void YMPME_Belt_Step(pme_belt *b, double dt);
	pme_belt_point YMPME_Belt_Path(const pme_belt *b, double distance);
	double YMPME_Belt_Length(const pme_belt *b);
#ifdef __cplusplus
}
#endif
#endif
