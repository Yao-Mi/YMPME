#include "control_rig.h"
#include <string.h>
void DroneControlRig_Init(drone_control_rig *r)
{
	memset(r, 0, sizeof(*r));
	DroneController_Init(&r->controller);
	r->seed = 42;
	r->imuDesc.mount = YMPME_Sensor_IdentityMount();
	r->imuDesc.gravity = YMPME_Vec3(0, -9.81f, 0);
	r->imuDesc.accelNoise = .015f;
	r->imuDesc.gyroNoise = .0005f;
	YMPME_SensorClock_Init(&r->sampleClock, 1.0 / 240, 0);
	YMPME_SensorClock_Init(&r->controlClock, 1.0 / 120, 0);
	for (int i = 0; i < 4; ++i)
	{
		YMPME_Encoder_Init(&r->encoder[i], PME_ENCODER_COUNTS_PER_TURN);
		r->motor[i] = sqrtf(1.6f * 9.81f / (4 * 11));
	}
}
int DroneControlRig_Step(drone_control_rig *r, pme_aircraft *a, const pme_aircraft_command *command,
						 pme_vec3 wind, float dt)
{
	pme_feedback_state state;
	pme_sensor_stamp stamp;
	if (YMPME_Aircraft_GetFeedback(a, &state) != PME_OK)
		return 0;
	if (a->lastStep > 0 && YMPME_SensorClock_Due(&r->sampleClock, state.time))
	{
		if (YMPME_SensorClock_Stamp(&r->sampleClock, state.time, &stamp) != PME_OK ||
			YMPME_IMU_Measure(&r->imuDesc, &state, &stamp, &r->seed, &r->imu) != PME_OK)
			return 0;
		for (int i = 0; i < 4; ++i)
			if (YMPME_Encoder_Measure(&r->encoder[i], a->rotorPhase[i], &stamp, &r->enc[i]) !=
				PME_OK)
				return 0;
		pme_wrench load = {a->load.force, a->load.torque};
		if (YMPME_ForceSensor_Measure(&load, &state, &r->imuDesc.mount, &stamp, &r->force) !=
			PME_OK)
			return 0;
	}
	if (r->imu.stamp.valid && YMPME_SensorClock_Due(&r->controlClock, state.time))
	{
		float elapsed =
			r->controlClock.started ? (float)(state.time - r->controlClock.lastTime) : 1.0f / 120;
		if (YMPME_SensorClock_Stamp(&r->controlClock, state.time, &stamp) != PME_OK ||
			!DroneController_Update(&r->controller, &state, &r->imu, command, elapsed, r->motor))
			return 0;
		++r->updates;
	}
	/* 电源切断立即传入执行器，不等待下一次控制采样。 */
	if (command->power <= 0)
		for (int i = 0; i < 4; ++i)
			r->motor[i] = 0;
	return YMPME_Aircraft_StepMotors(a, r->motor, wind, dt) == PME_OK;
}
