#include "control.h"
void DroneController_Init(drone_controller *c)
{
	*c = (drone_controller){1.6f, {.14f, .25f, .14f}, 0};
}
int DroneController_Update(drone_controller *c, const pme_feedback_state *s,
						   const pme_imu_sample *imu, const pme_aircraft_command *command, float dt,
						   float motor[4])
{
	if (!c || !s || !imu || !command || !motor || !isfinite(dt) || dt <= 0 || dt > .05f ||
		!imu->stamp.valid || imu->stamp.availableTime > s->time ||
		s->time - imu->stamp.time > .05 || !YMPME_Rotation_IsValid(s->rotation) ||
		(s->valid & (PME_FEEDBACK_POSE | PME_FEEDBACK_VELOCITY)) !=
			(PME_FEEDBACK_POSE | PME_FEEDBACK_VELOCITY))
		return 0;
	if (command->power <= 0)
	{
		for (int i = 0; i < 4; ++i)
			motor[i] = 0;
		c->verticalIntegral = 0;
		return 1;
	}
	pme_vec3 f = s->rotation.axis[2], r = s->rotation.axis[0], up = s->rotation.axis[1];
	float yaw = atan2f(f.x, f.z), sn = sinf(yaw), cs = cosf(yaw);
	float vx = 5 * (command->forward * sn + command->right * cs),
		  vz = 5 * (command->forward * cs - command->right * sn);
	float ax = YMPME_Clamp(2 * (vx - s->velocity.x), -5, 5),
		  az = YMPME_Clamp(2 * (vz - s->velocity.z), -5, 5);
	float targetPitch = YMPME_Clamp(atan2f(ax * sn + az * cs, 9.81f), -.45f, .45f),
		  targetRoll = YMPME_Clamp(-atan2f(ax * cs - az * sn, 9.81f), -.45f, .45f);
	float pitch = asinf(YMPME_Clamp(-f.y, -1, 1)), roll = atan2f(r.y, up.y);
	float pd = 4 * (targetPitch - pitch), rd = 4 * (targetRoll - roll), yd = command->turn * 1.2f;
	pme_vec3 rate =
		YMPME_Vec3(pd * cosf(roll) + yd * sinf(roll) * cosf(pitch),
				   -pd * sinf(roll) + yd * cosf(roll) * cosf(pitch), rd - yd * sinf(pitch));
	pme_vec3 w = imu->angularVelocity;
	pme_vec3 torque =
		YMPME_Vec3(c->inertia.x * 10 * (rate.x - w.x), c->inertia.y * 6 * (rate.y - w.y),
				   c->inertia.z * 10 * (rate.z - w.z));
	torque = YMPME_VecAdd(
		torque,
		YMPME_VecCross(w, YMPME_Vec3(c->inertia.x * w.x, c->inertia.y * w.y, c->inertia.z * w.z)));
	float error = command->climb * 2.5f - s->velocity.y;
	c->verticalIntegral = YMPME_Clamp(c->verticalIntegral + error * dt, -2, 2);
	float total = c->mass * (9.81f + YMPME_Clamp(4 * error + .6f * c->verticalIntegral, -6, 8)) /
				  fmaxf(.5f, up.y);
	for (int i = 0; i < 4; ++i)
	{
		float x = (i & 1) ? .45f : -.45f, z = (i & 2) ? .45f : -.45f,
			  spin = (i == 0 || i == 3) ? 1 : -1;
		float thrust =
			YMPME_Clamp(total / 4 - z * torque.x / (4 * .45f * .45f) +
							x * torque.z / (4 * .45f * .45f) - spin * torque.y / (4 * .03f),
						0, 11) *
			command->power;
		motor[i] = sqrtf(thrust / 11);
	}
	return 1;
}
