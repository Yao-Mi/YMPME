/* 可裁剪的三维刚体转动；与旧平移 World 分开步进，SI / 世界坐标。 */
#ifndef YMPME_RIGID_H
#define YMPME_RIGID_H
#include "../YMPME_Feedback.h"
#include "../YMPME_MathBase.h"
#ifdef __cplusplus
extern "C"
{
#endif
	typedef struct pme_quaternion_
	{
		float x, y, z, w;
	} pme_quaternion;
	typedef struct pme_rigid_body_
	{
		pme_vec3 position, velocity, angularVelocity;
		pme_quaternion orientation; /* 本体->世界；质心坐标系与惯量主轴重合。 */
		float mass, inverseMass;
		pme_vec3 inertia, inverseInertia; /* 本体坐标对角惯量，kg m^2。mass=0固定。 */
		pme_wrench load;				  /* 用户力、质心力矩；步进后清零，不含重力/约束。 */
		pme_feedback_state feedback;
		pme_vec3 previousVelocity, previousAngularVelocity;
	} pme_rigid_body;
	int YMPME_Quaternion_IsValid(pme_quaternion q);
	pme_quaternion YMPME_Quaternion_Identity(void);
	pme_quaternion YMPME_Quaternion_Multiply(pme_quaternion a, pme_quaternion b);
	pme_quaternion YMPME_Quaternion_Conjugate(pme_quaternion q);
	pme_result YMPME_Quaternion_AxisAngle(pme_vec3 axis, float angle, pme_quaternion *out);
	pme_rotation YMPME_Quaternion_Rotation(pme_quaternion q);
	pme_vec3 YMPME_Quaternion_Log(pme_quaternion q); /* 最短旋转向量，rad。 */
	pme_vec3 YMPME_Rotation_InverseApply(pme_rotation r, pme_vec3 v);
	pme_vec3 YMPME_Rigid_BoxInertia(float mass, pme_vec3 fullSize);
	pme_result YMPME_Rigid_Init(pme_rigid_body *body, float mass, pme_vec3 inertia,
								pme_vec3 position, pme_quaternion orientation);
	int YMPME_Rigid_IsValid(const pme_rigid_body *body);
	pme_vec3 YMPME_Rigid_InverseInertia(const pme_rigid_body *body, pme_vec3 worldVector);
	pme_result YMPME_Rigid_AddForceAt(pme_rigid_body *body, pme_vec3 force, pme_vec3 worldPoint);
	pme_result YMPME_Rigid_AddTorque(pme_rigid_body *body, pme_vec3 worldTorque);
	/* 冲量关于质心；JOINT 和自定义约束/接触求解器可复用，无分配。 */
	void YMPME_Rigid_Impulse(pme_rigid_body *body, pme_vec3 linear, pme_vec3 angular);
	/* 分阶段接口供JOINT调用：Begin -> 求解冲量 -> End。调用者先验证全部输入。
	 * 单独自由体使用Step；同一刚体不可再用旧World或AIRCRAFT重复积分。
	 * dt: (0,0.05]秒，推荐1/240；endTime为步末仿真时间。 */
	void YMPME_Rigid_Begin(pme_rigid_body *body, pme_vec3 gravity, float dt);
	void YMPME_Rigid_End(pme_rigid_body *body, float dt, double endTime);
	pme_result YMPME_Rigid_Step(pme_rigid_body *body, pme_vec3 gravity, float dt, double endTime);
#ifdef __cplusplus
}
#endif
#endif
