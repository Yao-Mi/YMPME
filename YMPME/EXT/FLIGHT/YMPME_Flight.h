#ifndef YMPME_FLIGHT_H
#define YMPME_FLIGHT_H
#include "../../CONFIG/YMPME_Features.h"
#if PME_ENABLE_FLIGHT
#include "../../CONFIG/YMPME_PubType.h"
/* 所有输入/输出使用同一坐标系。力 N，力矩 N*m，位置相对质心，单位 m。
 * 这里只计算载荷；不会隐式步进 World、施加重力或丢弃力矩。 */
typedef struct pme_flight_load_
{
	pme_vec3 force, torque;
} pme_flight_load;
#if PME_FLIGHT_ROTORS
typedef struct pme_rotor_desc_
{
	pme_vec3 position, axis; /* 质心到旋翼的位置、正推力轴（自动归一化）。 */
	float thrustFactor;      /* N/(rad/s)^2，非负；由具体旋翼参数给出。 */
	float reactionFactor;    /* N*m/(rad/s)^2，非负。 */
} pme_rotor_desc;
#endif
#if PME_FLIGHT_AERODYNAMICS
typedef struct pme_aero_desc_
{
	pme_vec3 position, liftAxis;                  /* 压力中心、期望升力方向（投影到垂直速度的平面）。 */
	float area, liftCoefficient, dragCoefficient; /* m^2、Cl（可负）、Cd（非负）。 */
} pme_aero_desc;
#endif
#ifdef __cplusplus
extern "C"
{
#endif
	pme_flight_load YMPME_Flight_ZeroLoad(void);
	/* 返回值失败时不修改输出；支持 out 与输入别名；函数均不分配内存。 */
	pme_result YMPME_Flight_AddLoads(const pme_flight_load *a, const pme_flight_load *b,
	                                 pme_flight_load *out);
	pme_result YMPME_Flight_LoadAtPoint(pme_vec3 force, pme_vec3 position, pme_vec3 freeTorque,
	                                    pme_flight_load *out);
#if PME_FLIGHT_ROTORS
	/* 带符号轴速 rad/s：反转只翻转反扭矩，推力方向由 axis 决定。
	 * 简化静态平方律，不模拟电机滞后、入流、变距或反桨推力。 */
	pme_result YMPME_Flight_RotorLoad(const pme_rotor_desc *rotor, float angularSpeed, pme_flight_load *out);
#endif
#if PME_FLIGHT_AERODYNAMICS
	/* airVelocity = 物体相对空气的速度 (m/s)；density 为 kg/m^3，非负。
	 * Cl/Cd 由应用提供；不自动模拟攻角、失速、马赫数或翼型查表。
	 * liftAxis 平行于速度且请求非零升力时，方向不唯一，返回参数错误。 */
	pme_result YMPME_Flight_AeroLoad(const pme_aero_desc *aero, pme_vec3 airVelocity, float density,
	                                 pme_flight_load *out);
#endif
#ifdef __cplusplus
}
#endif
#endif /* PME_ENABLE_FLIGHT */
#endif
