#ifndef DEMO_CONTROL_RIG_H
#define DEMO_CONTROL_RIG_H
#include "control.h"
#include "SENSOR/ENCODER/YMPME_Encoder.h"
#include "SENSOR/FORCE/YMPME_ForceSensor.h"
typedef struct drone_control_rig_
{
	drone_controller controller;
	pme_imu_desc imuDesc;
	pme_imu_sample imu;
	pme_encoder encoder[4];
	pme_encoder_sample enc[4];
	pme_force_sample force;
	pme_sensor_clock sampleClock, controlClock;
	float motor[4];
	uint32_t seed;
	unsigned updates;
} drone_control_rig;
void DroneControlRig_Init(drone_control_rig *rig);
int DroneControlRig_Step(drone_control_rig *rig, pme_aircraft *aircraft,
						 const pme_aircraft_command *command, pme_vec3 wind, float dt);
#endif
