/* 固定轴20/30齿、模数0.12m的传动预设，含理想约束/紧耦合/回差模式。
 * Profile 单独支持18..40齿，module>0(m)，capacity>=teeth*23；失败返回0、不写点数组。
 * ResetMode 初始化全部状态；角度rad、转速rad/s、扭矩N*m。target/motor/loadTorque可步间修改。
 * 每实例含约32KiB齿形缓存，初始化时建立，独立实例不共享可变状态；无堆分配。
 * Step: 0<dt<=.05s，内部碰撞子步<=1/1200s，成功1失败0；需有效的已初始化状态。
 * distance可调但需保持固定双轴近似；不提供通用旋转刚体/任意齿轮副。
 * Getter/CollisionStep 亦要求已初始化实例，状态数组由实例持有。 */
#ifndef YMPME_GEAR_H
#define YMPME_GEAR_H
#include "../../CORE/YMPME_MathBase.h"
#define PME_GEAR_PI 3.14159265358979323846
#define PME_GEAR_PROFILE_MAX 1024
#ifdef __cplusplus
extern "C"
{
#endif
	typedef struct pme_gear_point_
	{
		double x, y;
	} pme_gear_point;
	typedef enum pme_gear_mode_
	{
		PME_GEAR_CONSTRAINT,
		PME_GEAR_TIGHT,
		PME_GEAR_BACKLASH
	} pme_gear_mode;
	typedef struct pme_gear_pair_
	{
		double angle, omega, target, torque;
		int motor;
		pme_gear_point profile[2][PME_GEAR_PROFILE_MAX];
		int profileReady;
		pme_gear_mode mode;
		double angleB, omegaB, distance, loadTorque;
		double contactTorque, maxPenetration;
		int contacts, candidates, pointCount;
		pme_gear_point contactPoint[8];
	} pme_gear_pair;
	int YMPME_Gear_Profile(int teeth, double module, pme_gear_point *points, int capacity);
	void YMPME_Gear_Reset(pme_gear_pair *pair);
	void YMPME_Gear_ResetMode(pme_gear_pair *pair, pme_gear_mode mode);
	int YMPME_Gear_CollisionStep(pme_gear_pair *pair, double dt);
	double YMPME_Gear_DrivenOmega(const pme_gear_pair *pair);
	int YMPME_Gear_Step(pme_gear_pair *pair, double dt);
	double YMPME_Gear_DrivenAngle(const pme_gear_pair *pair);
#ifdef __cplusplus
}
#endif
#endif
