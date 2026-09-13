#ifndef YMPME_AIRCRAFT_H
#define YMPME_AIRCRAFT_H
#include "../../CORE/YMPME_MathBase.h"
#include "../../CORE/YMPME_Feedback.h"
#include "../../EXT/FLIGHT/YMPME_Flight.h"
#ifdef __cplusplus
extern "C"
{
#endif
	/* 游戏用受控飞行预设，Y向上、局部+Z向前、+X向右；SI单位、角度rad。
	 * 独立推进位置及四元数姿态，不能再交由 World 重复积分。不含机体互撞。
	 * 无GRE/SDL/堆分配；状态由调用方持有，正常使用只修改 command 与 wind。
	 * 鸟为等效翼面+周期推力近似，非生物肌肉/羽毛或高保真流体模型。 */
	typedef enum pme_aircraft_kind_
	{
		PME_AIRCRAFT_DRONE,
		PME_AIRCRAFT_EAGLE
	} pme_aircraft_kind;
	typedef struct pme_aircraft_command_
	{
		float forward, right, climb, turn; /* [-1,1]：前后、横移/滚转、升降/俯仰、转向。 */
		float power;					   /* [0,1]；0关闭旋翼/振翅，鸟仍有被动翼面升阻力。 */
	} pme_aircraft_command;
	typedef struct pme_aircraft_
	{
		pme_aircraft_kind kind;
		pme_vec3 position, velocity, angularVelocity; /* 角速度为局部坐标rad/s。 */
		float q[4];									  /* 局部到世界，单位四元数x,y,z,w。 */
		float mass;
		pme_vec3 inertia;
		float rotorSpeed[4], rotorPhase[4], wingPhase, wingAngle;
		float lift, thrust, airspeed;
		pme_flight_load load; /* 世界坐标总非重力载荷，地面反力不计入。 */
		double time;
		pme_vec3 acceleration, angularAcceleration; /* 世界坐标，完整物理步的速度差分。 */
		float lastStep;
		int grounded;
	} pme_aircraft;
	/* Init失败不改状态，初始位置必须y>=.3；无人机静止、鹰初速+Z 10m/s。 */
	pme_result YMPME_Aircraft_Init(pme_aircraft *aircraft, pme_aircraft_kind kind,
								   pme_vec3 position);
	/* 已初始化实例；全部输入有限；0<dt<=1/120s。非法输入或数值失败不改实例。
	 * 旋翼/翼面复用EXT/FLIGHT；内置平面y=0落地支承，地面中心高度.3m。 */
	pme_result YMPME_Aircraft_Step(pme_aircraft *aircraft, const pme_aircraft_command *command,
								   pme_vec3 wind, float dt);
	/* 绕过所有内置速度/姿态控制和混控；仅DRONE可用。motor[4]为[0,1]转速比例，
	 * 1对应sqrt(11/.0001) rad/s，保留电机响应滞后与真实旋翼载荷。
	 * 0/1/2/3位置为(-x,-z)/(+x,-z)/(-x,+z)/(+x,+z)，转向+/-/-/+。 */
	pme_result YMPME_Aircraft_StepMotors(pme_aircraft *aircraft, const float motor[4],
										 pme_vec3 wind, float dt);
	pme_result YMPME_Aircraft_GetFeedback(const pme_aircraft *aircraft, pme_feedback_state *out);
	/* 已初始化实例的无分配坐标转换，local可为点的相对偏移或方向。 */
	pme_vec3 YMPME_Aircraft_Rotate(const pme_aircraft *aircraft, pme_vec3 local);
#ifdef __cplusplus
}
#endif
#endif
