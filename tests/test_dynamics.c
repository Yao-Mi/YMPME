#include "YMPME.h"
#if !PME_ENABLE_DYNAMICS || !PME_ENABLE_JOINT || !PME_ENABLE_RIGID
#error "Dynamics target must export its required feature macros"
#endif
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define V YMPME_Vec3
#define CHECK(x)                                                                                   \
	do                                                                                             \
	{                                                                                              \
		if (!(x))                                                                                  \
		{                                                                                          \
			fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #x);                                \
			exit(1);                                                                               \
		}                                                                                          \
	} while (0)
static float norm(pme_vec3 v) { return sqrtf(YMPME_VecDot(v, v)); }
static pme_rigid_body body(float m, pme_vec3 p)
{
	pme_rigid_body b;
	CHECK(YMPME_Rigid_Init(&b, m, V(.1f, .1f, .1f), p, YMPME_Quaternion_Identity()) == PME_OK);
	return b;
}
static pme_dynamics world(pme_rigid_body *b, pme_collider *c, pme_dynamic_contact *ct)
{
	pme_dynamics w = YMPME_Dynamics_Default();
	w.bodies = b;
	w.bodyCount = 2;
	w.colliders = c;
	w.colliderCount = 2;
	w.contacts = ct;
	w.contactCapacity = 16;
	return w;
}
static void step(pme_dynamics *w)
{
	CHECK(YMPME_Dynamics_Step(w, 1.f / 240, w->time + 1.0 / 240) == PME_OK);
}
static void conservation(void)
{
	pme_rigid_body b[2] = {body(2, V(-.5f, 0, 0)), body(1, V(.5f, 0, 0))};
	pme_collider c[2] = {YMPME_Dynamics_Collider(0, PME_COLLIDER_SPHERE),
						 YMPME_Dynamics_Collider(1, PME_COLLIDER_SPHERE)};
	pme_dynamic_contact ct[16];
	pme_dynamics w = world(b, c, ct);
	w.gravity = V(0, 0, 0);
	b[0].velocity = V(1, 1, 0);
	b[1].velocity = V(-1, 0, 0);
	c[0].restitution = c[1].restitution = 1;
	step(&w);
	CHECK(w.contactCount == 1);
	CHECK(b[0].angularVelocity.z < -.1f && b[1].angularVelocity.z < -.1f);
	pme_vec3 momentum = YMPME_VecAdd(YMPME_VecScale(b[0].velocity, 2), b[1].velocity);
	CHECK(norm(YMPME_VecSub(momentum, V(1, 2, 0))) < 1e-5f);
	pme_vec3 L = V(0, 0, 0);
	for (int i = 0; i < 2; ++i)
		L = YMPME_VecAdd(
			L, YMPME_VecAdd(YMPME_VecCross(b[i].position, YMPME_VecScale(b[i].velocity, b[i].mass)),
							YMPME_VecScale(b[i].angularVelocity, .1f)));
	CHECK(fabsf(L.z + 1) < 1e-5f);
	CHECK(norm(YMPME_VecAdd(ct[0].reactionA.force, ct[0].reactionB.force)) < 1e-6f);
	pme_vec3 torque = YMPME_VecAdd(
		YMPME_VecAdd(ct[0].reactionA.torque, YMPME_VecCross(ct[0].originA, ct[0].reactionA.force)),
		YMPME_VecAdd(ct[0].reactionB.torque, YMPME_VecCross(ct[0].originB, ct[0].reactionB.force)));
	CHECK(norm(torque) < 1e-5f);
	CHECK(norm(ct[0].tangentImpulse) <= ct[0].friction * ct[0].normalImpulse + 1e-6f);
}
static void floor_and_rolling(void)
{
	pme_rigid_body b[2] = {body(0, V(0, 0, 0)), body(1, V(0, .5f, 0))};
	pme_collider c[2] = {YMPME_Dynamics_Collider(0, PME_COLLIDER_PLANE),
						 YMPME_Dynamics_Collider(1, PME_COLLIDER_SPHERE)};
	pme_dynamic_contact ct[16];
	pme_dynamics w = world(b, c, ct);
	b[1].velocity = V(1, 0, 0);
	for (int i = 0; i < 720; ++i)
		step(&w);
	CHECK(fabsf(b[1].position.y - .5f) < .001f);
	CHECK(b[1].angularVelocity.z < -.5f);
	CHECK(fabsf(b[1].velocity.x + .5f * b[1].angularVelocity.z) < .002f);
	CHECK(fabsf(ct[0].reactionB.force.y - 9.81f) < .02f);
	/* Four coplanar box support contacts, finite angular solution. */
	b[1] = body(1, V(0, .5f, 0));
	b[0] = body(0, V(0, 0, 0));
	c[1].type = PME_COLLIDER_BOX;
	w.time = 0;
	for (int i = 0; i < 480; ++i)
		step(&w);
	CHECK(w.contactCount == 4);
	CHECK(norm(b[1].angularVelocity) < .01f);
	CHECK(fabsf(b[1].position.y - .5f) < .002f);
}
static void geometry_and_errors(void)
{
	pme_rigid_body b[2] = {body(0, V(0, 0, 0)), body(1, V(0, .9f, 0))};
	pme_collider c[2] = {YMPME_Dynamics_Collider(0, PME_COLLIDER_BOX),
						 YMPME_Dynamics_Collider(1, PME_COLLIDER_SPHERE)};
	pme_dynamic_contact ct[16];
	pme_dynamics w = world(b, c, ct);
	w.gravity = V(0, 0, 0);
	c[0].halfSize = V(1, .2f, .2f);
	CHECK(YMPME_Quaternion_AxisAngle(V(0, 0, 1), 1.57079632679f, &b[0].orientation) == PME_OK);
	step(&w);
	CHECK(w.contactCount == 1);
	CHECK(ct[0].penetration > .49f);
	CHECK(YMPME_VecFinite(ct[0].normal));
	pme_rigid_body saved[2];
	memcpy(saved, b, sizeof b);
	double old = w.time;
	w.contactCapacity = 0;
	CHECK(YMPME_Dynamics_Step(&w, 1.f / 240, w.time + 1. / 240) == PME_CAPACITY_EXCEEDED);
	CHECK(!memcmp(saved, b, sizeof b) && old == w.time);
	w.contactCapacity = 16;
	c[0].group = c[1].group = -3;
	step(&w);
	CHECK(w.contactCount == 0);
	c[0].group = c[1].group = 0;
	c[0].mask = 0;
	step(&w);
	CHECK(w.contactCount == 0);
	c[0].group = c[1].group = 3;
	step(&w);
	CHECK(w.contactCount == 1);
	c[1].type = (pme_collider_type)99;
	memcpy(saved, b, sizeof b);
	CHECK(YMPME_Dynamics_Step(&w, 1.f / 240, w.time + 1. / 240) == PME_INVALID_ARGUMENT);
	CHECK(!memcmp(saved, b, sizeof b));
	c[1].type = PME_COLLIDER_SPHERE;
	c[1].radius = NAN;
	CHECK(YMPME_Dynamics_Step(&w, 1.f / 240, w.time + 1. / 240) == PME_INVALID_ARGUMENT);
	CHECK(!memcmp(saved, b, sizeof b));
}
static void joint_contact(void)
{
	/* Motor force into a sphere attached to a fixed wall: contact must oppose the
	 * motor within the SAME solve, not move through a post-integrated obstacle. */
	pme_rigid_body b[2] = {body(0, V(1, 0, 0)), body(1, V(0, 0, 0))};
	pme_collider c[2] = {YMPME_Dynamics_Collider(0, PME_COLLIDER_SPHERE),
						 YMPME_Dynamics_Collider(1, PME_COLLIDER_SPHERE)};
	pme_dynamic_contact ct[16];
	pme_dynamics w = world(b, c, ct);
	w.gravity = V(0, 0, 0);
	pme_joint j;
	pme_quaternion q;
	CHECK(YMPME_Quaternion_AxisAngle(V(0, 1, 0), 1.57079632679f, &q) == PME_OK);
	CHECK(YMPME_Joint_Init(&j, PME_JOINT_PRISMATIC, b, 2, PME_JOINT_WORLD, 1, V(0, 0, 0), q) ==
		  PME_OK);
	CHECK(YMPME_Joint_SetMotor(&j, (pme_joint_motor){PME_JOINT_MOTOR_EFFORT, 0, 5, 5}) == PME_OK);
	w.joints = &j;
	w.jointCount = 1;
	for (int i = 0; i < 240; ++i)
		step(&w);
	CHECK(fabsf(b[1].position.x) < .001f);
	CHECK(fabsf(j.feedback.motorEffort - 5) < 1e-4f);
	CHECK(fabsf(ct[0].reactionB.force.x + 5) < .01f);
}
static void off_center_normal(void)
{
	pme_rigid_body b[2] = {body(1, V(0, 0, 0)), body(1, V(1, .3f, 0))};
	pme_collider c[2] = {YMPME_Dynamics_Collider(0, PME_COLLIDER_BOX),
						 YMPME_Dynamics_Collider(1, PME_COLLIDER_SPHERE)};
	c[0].friction = c[1].friction = 0;
	pme_dynamic_contact ct[16];
	pme_dynamics w = world(b, c, ct);
	w.gravity = V(0, 0, 0);
	b[1].velocity = V(-1, 0, 0);
	step(&w);
	CHECK(w.contactCount == 1 && b[0].angularVelocity.z > .1f);
	CHECK(norm(b[1].angularVelocity) < 1e-6f);
	CHECK(fabsf(ct[0].point.x - .5f) < 1e-5f && fabsf(ct[0].point.y - .3f) < 1e-5f);
	CHECK(ct[0].reactionA.torque.z > 0);
}
int main(void)
{
	conservation();
	off_center_normal();
	floor_and_rolling();
	geometry_and_errors();
	joint_contact();
	puts("DYNAMICS PASS: momentum, torque, rolling, support, filters, capacity, joint/contact "
		 "coupling");
	return 0;
}
