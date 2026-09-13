#ifndef JOINT_LAB_CONTROL_H
#define JOINT_LAB_CONTROL_H
#include "MODEL/PENDULUM/YMPME_Pendulum.h"
#include "SENSOR/ENCODER/YMPME_Encoder.h"
typedef struct joint_lab_state_
{
	pme_pendulum pendulum;
	pme_rigid_body fixedBody, sliderBody;
	pme_joint fixedJoint, sliderJoint;
	pme_encoder encoder;
	pme_encoder_sample sample;
	pme_sensor_clock clock;
	uint16_t lastEncoder;
	int64_t accumulatedTicks;
	float target;
	int servo, limits, extraLoad, automatic;
} joint_lab_state;
int JointLab_Init(joint_lab_state *s);
int JointLab_Step(joint_lab_state *s);
#endif
