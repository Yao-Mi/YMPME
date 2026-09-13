/* 可复用电机单摆：质量忽略的杆 + 球形负载 + Z轴转动关节。 */
#ifndef YMPME_PENDULUM_H
#define YMPME_PENDULUM_H
#include "../../JOINT/YMPME_Joint.h"
#ifdef __cplusplus
extern "C"
{
#endif
	typedef struct pme_pendulum_config_
	{
		pme_vec3 anchor;
		float length, mass, radius, initialAngle;
	} pme_pendulum_config;
	typedef struct pme_pendulum_
	{
		pme_pendulum_config config;
		pme_rigid_body body;
		pme_joint joint;
		double time;
	} pme_pendulum;
	pme_pendulum_config YMPME_Pendulum_DefaultConfig(void);
	pme_result YMPME_Pendulum_Init(pme_pendulum *model, const pme_pendulum_config *config);
	/* 电机参数通过joint设置；控制器/编码器在应用侧，模型不依赖SENSOR/GRE。 */
	pme_result YMPME_Pendulum_Step(pme_pendulum *model, pme_vec3 gravity, float dt);
#ifdef __cplusplus
}
#endif
#endif
