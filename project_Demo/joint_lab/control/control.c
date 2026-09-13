#include "control.h"
#include <string.h>
#define V(x, y, z) YMPME_Vec3(x, y, z)
int JointLab_Init(joint_lab_state *s)
{
	pme_pendulum_config c = YMPME_Pendulum_DefaultConfig();
	pme_quaternion frame;
	pme_sensor_stamp stamp = {0, 0, 0, 1};
	memset(s, 0, sizeof(*s));
	c.initialAngle = -.55f;
	s->target = .6f;
	s->servo = s->limits = 1;
	if (YMPME_Pendulum_Init(&s->pendulum, &c) != PME_OK)
		return 0;
	if (YMPME_Rigid_Init(&s->fixedBody, 2, YMPME_Rigid_BoxInertia(2, V(1.8f, .12f, .25f)),
						 V(.45f, 1.8f, 0), YMPME_Quaternion_Identity()) != PME_OK)
		return 0;
	if (YMPME_Joint_Init(&s->fixedJoint, PME_JOINT_FIXED, &s->fixedBody, 1, PME_JOINT_WORLD, 0,
						 V(-.45f, 1.8f, 0), YMPME_Quaternion_Identity()) != PME_OK)
		return 0;
	if (YMPME_Rigid_Init(&s->sliderBody, 2, YMPME_Rigid_BoxInertia(2, V(.6f, .4f, .6f)),
						 V(0, 1.5f, 0), YMPME_Quaternion_Identity()) != PME_OK)
		return 0;
	if (YMPME_Quaternion_AxisAngle(V(0, 1, 0), 1.570796327f, &frame) != PME_OK)
		return 0;
	if (YMPME_Joint_Init(&s->sliderJoint, PME_JOINT_PRISMATIC, &s->sliderBody, 1, PME_JOINT_WORLD,
						 0, V(0, 1.5f, 0), frame) != PME_OK)
		return 0;
	if (YMPME_Joint_SetLimit(&s->sliderJoint, (pme_joint_limit){1, -1.2f, 1.2f}) != PME_OK)
		return 0;
	if (YMPME_Encoder_Init(&s->encoder, 0) != PME_OK ||
		YMPME_SensorClock_Init(&s->clock, 1.0 / 240, 0) != PME_OK)
		return 0;
	if (YMPME_Encoder_Measure(&s->encoder, c.initialAngle, &stamp, &s->sample) != PME_OK)
		return 0;
	s->lastEncoder = s->sample.angle;
	return 1;
}
int JointLab_Step(joint_lab_state *s)
{
	const float dt = 1.f / 240;
	const pme_vec3 gravity = {0, -9.81f, 0};
	pme_joint *hinge = &s->pendulum.joint;
	pme_sensor_stamp stamp;
	float angle = hinge->feedback.coordinate, speed = hinge->feedback.speed;
	double time = s->pendulum.time + dt;
	float target = s->automatic ? .95f * sinf((float)time * .7f) : s->target;
	/* Application PD + known gravity compensation; physics only receives torque. */
	float effort = 40 * (target - angle) - 10 * speed +
				   s->pendulum.config.mass * 9.81f * s->pendulum.config.length * sinf(angle);
	if (YMPME_Joint_SetMotor(
			hinge, (pme_joint_motor){s->servo ? PME_JOINT_MOTOR_EFFORT : PME_JOINT_MOTOR_OFF, 0,
									 effort, 25}) != PME_OK)
		return 0;
	if (YMPME_Joint_SetLimit(hinge, (pme_joint_limit){s->limits, -1.3962634f, 1.3962634f}) !=
		PME_OK)
		return 0;
	if (YMPME_Pendulum_Step(&s->pendulum, gravity, dt) != PME_OK)
		return 0;
	if (YMPME_Rigid_AddForceAt(&s->fixedBody, V(0, s->extraLoad ? -30 : 0, 0), V(1.35f, 1.8f, 0)) !=
		PME_OK)
		return 0;
	if (YMPME_Joint_Step(&s->fixedBody, 1, &s->fixedJoint, 1, gravity, dt, 32, time) != PME_OK)
		return 0;
	effort = 50 * (sinf((float)time * .8f) - s->sliderJoint.feedback.coordinate) -
			 12 * s->sliderJoint.feedback.speed;
	if (YMPME_Joint_SetMotor(&s->sliderJoint,
							 (pme_joint_motor){PME_JOINT_MOTOR_EFFORT, 0, effort, 20}) != PME_OK)
		return 0;
	if (YMPME_Joint_Step(&s->sliderBody, 1, &s->sliderJoint, 1, gravity, dt, 32, time) != PME_OK)
		return 0;
	if (YMPME_SensorClock_Due(&s->clock, time))
	{
		uint16_t now, last;
		int16_t delta;
		if (YMPME_SensorClock_Stamp(&s->clock, time, &stamp) != PME_OK ||
			YMPME_Encoder_Measure(&s->encoder, hinge->feedback.coordinate, &stamp, &s->sample) !=
				PME_OK)
			return 0;
		now = s->sample.angle;
		last = s->lastEncoder;
		delta = (int16_t)(now - last);
		s->accumulatedTicks += delta;
		s->lastEncoder = now;
	}
	return 1;
}
