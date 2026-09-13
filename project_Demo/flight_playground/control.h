#ifndef DEMO_DRONE_CONTROL_H
#define DEMO_DRONE_CONTROL_H
#include "MODEL/AIRCRAFT/YMPME_Aircraft.h"
#include "SENSOR/IMU/YMPME_IMU.h"
/* 示例控制器仅接收反馈数据和驾驶命令，不接受/修改物理对象。
 * 理想位姿/速度反馈 + IMU陀螺闭环；不声称仅靠IMU实现定位或姿态估计。 */
typedef struct drone_controller_
{
	float mass;
	pme_vec3 inertia;
	float verticalIntegral;
} drone_controller;
void DroneController_Init(drone_controller *controller);
int DroneController_Update(drone_controller *controller, const pme_feedback_state *state,
						   const pme_imu_sample *imu, const pme_aircraft_command *command, float dt,
						   float motor[4]);
#endif
