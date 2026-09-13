#include "grasp.h"
#include <string.h>
#define V YMPME_Vec3
#define PI 3.14159265358979323846f
#define HALF_HEIGHT .40f
#define RADIUS .22f
static int initBody(pme_rigid_body *b, float mass, pme_vec3 size, pme_vec3 pos)
{
	return YMPME_Rigid_Init(b, mass, YMPME_Rigid_BoxInertia(mass, size), pos,
							YMPME_Quaternion_Identity()) == PME_OK;
}
size_t Grasp_CellCount(int sensor) { return sensor >= 0 && sensor < 2 ? 128 : 0; }
int Grasp_Init(grasp_lab *s, int mode) { return Grasp_InitShape(s, mode, 0); }
int Grasp_InitShape(grasp_lab *s, int mode, int shape)
{
	if (!s || shape < 0 || shape > 2)
		return 0;
	memset(s, 0, sizeof(*s));
	s->mode = !!mode;
	s->shape = shape;
	s->automatic = 1;
	s->motors = 1;
	s->friction = .85f;
	s->motorLimit = 2.5f;
	if (!initBody(&s->bodies[0], 2, V(.9f, .12f, .3f), V(0, 1, 0)) ||
		!initBody(&s->bodies[1], .25f, V(.1f, .8f, .3f), V(-.36f, .55f, 0)) ||
		!initBody(&s->bodies[2], .25f, V(.1f, .8f, .3f), V(.36f, .55f, 0)) ||
		!initBody(&s->bodies[3], .30f, V(.44f, .44f, .44f), V(0, .221f, 0)) ||
		!initBody(&s->bodies[4], 0, V(1, 1, 1), V(0, 0, 0)))
		return 0;
	float inertia = .4f * .30f * RADIUS * RADIUS;
	if (YMPME_Rigid_Init(&s->bodies[3], .30f, V(inertia, inertia, inertia), V(0, .221f, 0),
						 YMPME_Quaternion_Identity()) != PME_OK)
		return 0;
	if (shape)
	{
		if (!Grasp_Geometry(&s->geometry, shape, .30f) ||
			YMPME_Rigid_Init(&s->bodies[3], .30f, s->geometry.inertia, V(0, .201f, 0),
							 YMPME_Quaternion_Identity()) != PME_OK)
			return 0;
	}
	pme_quaternion q;
	if (YMPME_Quaternion_AxisAngle(V(1, 0, 0), -PI / 2, &q) != PME_OK ||
		YMPME_Joint_Init(&s->joints[0], PME_JOINT_PRISMATIC, s->bodies, 5, PME_JOINT_WORLD, 0,
						 V(0, 1, 0), q) != PME_OK)
		return 0;
	for (int i = 1; i <= 2; ++i)
	{
		if (YMPME_Joint_Init(&s->joints[i], PME_JOINT_REVOLUTE, s->bodies, 5, 0, (size_t)i,
							 V(i == 1 ? -.36f : .36f, .95f, 0),
							 YMPME_Quaternion_Identity()) != PME_OK ||
			YMPME_Joint_SetLimit(&s->joints[i], (pme_joint_limit){1, -.65f, .65f}) != PME_OK)
			return 0;
	}
	if (YMPME_Joint_SetLimit(&s->joints[0], (pme_joint_limit){1, 0, .75f}) != PME_OK)
		return 0;
	for (int i = 0; i < 4; ++i)
	{
		s->colliders[i] = YMPME_Dynamics_Collider((size_t)i + 1, i < 2	  ? PME_COLLIDER_BOX
																 : i == 2 ? PME_COLLIDER_SPHERE
																		  : PME_COLLIDER_PLANE);
		s->colliders[i].friction = s->friction;
		s->colliders[i].category = i < 2 ? 1 : i == 2 ? 2 : 4;
		s->colliders[i].mask = i < 2	? 2
							   : i == 2 ? 5
										: 2; /* ignore fingers against each other / floor */
		s->colliders[i].halfSize = V(.05f, HALF_HEIGHT, .15f);
		s->colliders[i].radius = RADIUS;
	}
	if (shape)
	{
		s->colliders[2].type = shape == 1 ? PME_COLLIDER_BOX : PME_COLLIDER_MESH;
		s->colliders[2].halfSize = V(.2f, .2f, .2f);
		s->colliders[2].mesh = &s->geometry.mesh;
	}
	s->world = YMPME_Dynamics_Default();
	s->world.bodies = s->bodies;
	s->world.bodyCount = 5;
	s->world.joints = s->joints;
	s->world.jointCount = 3;
	s->world.colliders = s->colliders;
	s->world.colliderCount = 4;
	s->world.contacts = s->contacts;
	s->world.contactCapacity = 32;
	s->world.iterations = 64;
	for (int i = 0; i < 2; ++i)
	{
		for (size_t k = 0; k < GRASP_CELLS; ++k)
			s->patches[i][k].area = .8f * .3f / 128;
		s->tactile[i].cells = s->cells[i];
		s->tactile[i].capacity = GRASP_CELLS;
	}
	return 1;
}
static void patchGeometry(grasp_lab *s, const pme_feedback_state pose[2])
{
	for (int sensor = 0; sensor < 2; ++sensor)
	{
		s->sampleRotation[sensor] = pose[sensor].rotation;
		pme_vec3 n = YMPME_Rotation_Apply(pose[sensor].rotation, V(sensor == 0 ? 1 : -1, 0, 0));
		for (size_t cell = 0; cell < 128; ++cell)
		{
			int x = (int)cell % 8, y = (int)cell / 8;
			pme_vec3 p = V(sensor == 0 ? .05f : -.05f, (.5f - (y + .5f) / 16) * .8f,
						   ((x + .5f) / 8 - .5f) * .3f);
			s->patches[sensor][cell].position =
				YMPME_VecAdd(pose[sensor].position, YMPME_Rotation_Apply(pose[sensor].rotation, p));
			s->patches[sensor][cell].normal = n;
		}
	}
}
static void assign(grasp_lab *s, int sensor, pme_vec3 p, pme_vec3 force,
				   const pme_feedback_state pose[2])
{
	p = YMPME_Rotation_InverseApply(pose[sensor].rotation, YMPME_VecSub(p, pose[sensor].position));
	int x = (int)floorf((p.z / .3f + .5f) * 8), y = (int)floorf((.5f - p.y / .8f) * 16);
	x = x < 0 ? 0 : x > 7 ? 7 : x;
	y = y < 0 ? 0 : y > 15 ? 15 : y;
	size_t cell = (size_t)y * 8 + x;
	s->forces[sensor][cell] = YMPME_VecAdd(s->forces[sensor][cell], force);
}
int Grasp_Step(grasp_lab *s)
{
	float t = (float)s->world.time;
	float left, right;
	if (s->automatic)
	{
		s->targetHeight = s->mode ? 0
								  : (t < 2	 ? 0
									 : t < 5 ? .65f * (t - 2) / 3
									 : t < 7 ? .65f
									 : t < 9 ? .65f
											 : 0);
		s->targetAngle = s->mode ? .34f : (t < .5f ? -.10f : t < 7 ? .40f : -.35f);
	}
	left = s->targetAngle;
	right = s->mode ? .55f : -s->targetAngle;
	float liftSpeed =
		YMPME_Clamp(5 * (s->targetHeight - s->joints[0].feedback.coordinate), -.3f, .3f);
	if (YMPME_Joint_SetMotor(&s->joints[0], (pme_joint_motor){s->motors ? PME_JOINT_MOTOR_SPEED
																		: PME_JOINT_MOTOR_OFF,
															  liftSpeed, 0, 80}) != PME_OK)
		return 0;
	for (int i = 1; i <= 2; ++i)
	{
		float speed = YMPME_Clamp(7 * ((i == 1 ? left : right) - s->joints[i].feedback.coordinate),
								  -.65f, .65f);
		if (YMPME_Joint_SetMotor(&s->joints[i], (pme_joint_motor){s->motors ? PME_JOINT_MOTOR_SPEED
																			: PME_JOINT_MOTOR_OFF,
																  speed, 0, s->motorLimit}) !=
			PME_OK)
			return 0;
	}
	for (int i = 0; i < 4; ++i)
		s->colliders[i].friction = s->friction;
	pme_feedback_state pose[2];
	for (int i = 0; i < 2; ++i)
	{
		pose[i] = s->bodies[i + 1].feedback;
		pose[i].position = s->bodies[i + 1].position;
		pose[i].rotation = YMPME_Quaternion_Rotation(s->bodies[i + 1].orientation);
	}
	if (YMPME_Dynamics_Step(&s->world, GRASP_DT, (s->steps + 1) / 240.0) != PME_OK)
		return 0;
	++s->steps;
	memset(s->forces, 0, sizeof(s->forces));
	memset(s->fingerForce, 0, sizeof(s->fingerForce));
	s->gripForce = s->groundForce = V(0, 0, 0);
	s->objectContact = (pme_wrench){0};
	patchGeometry(s, pose);
	for (size_t i = 0; i < s->world.contactCount; ++i)
	{
		const pme_dynamic_contact *c = &s->contacts[i];
		if (c->bodyA >= 1 && c->bodyA <= 2)
			assign(s, (int)c->bodyA - 1, c->point, c->reactionA.force, pose);
		if (c->bodyB >= 1 && c->bodyB <= 2)
			assign(s, (int)c->bodyB - 1, c->point, c->reactionB.force, pose);
		pme_wrench f = c->bodyA == GRASP_OBJECT ? c->reactionA : c->reactionB;
		s->objectContact.force = YMPME_VecAdd(s->objectContact.force, f.force);
		s->objectContact.torque = YMPME_VecAdd(s->objectContact.torque, f.torque);
		if (c->bodyA == GRASP_GROUND || c->bodyB == GRASP_GROUND)
			s->groundForce = YMPME_VecAdd(
				s->groundForce, c->bodyA == GRASP_OBJECT ? c->reactionA.force : c->reactionB.force);
		else
		{
			int finger = (int)(c->bodyA == GRASP_OBJECT ? c->bodyB : c->bodyA) - 1;
			pme_vec3 f = c->bodyA == GRASP_OBJECT ? c->reactionB.force : c->reactionA.force;
			s->fingerForce[finger] = YMPME_VecAdd(s->fingerForce[finger], f);
			s->gripForce = YMPME_VecSub(s->gripForce, f);
		}
	}
	pme_sensor_stamp stamp = {s->world.time, s->world.time, s->steps, 1};
	for (int i = 0; i < 2; ++i)
		if (YMPME_SurfaceTactile_Measure(s->patches[i], s->forces[i], Grasp_CellCount(i), &stamp,
										 pose[i].position, 1, &s->tactile[i]) != PME_OK)
			return 0;
	return 1;
}
float Grasp_NormalForce(const grasp_lab *s, int sensor)
{
	float sum = 0;
	for (size_t i = 0; i < Grasp_CellCount(sensor); ++i)
		sum += s->cells[sensor][i].normalForce;
	return sum;
}
