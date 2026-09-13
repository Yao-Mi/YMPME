#include "YMPME_Pendulum.h"
#include <string.h>
pme_pendulum_config YMPME_Pendulum_DefaultConfig(void)
{
	return (pme_pendulum_config){{0, 2.5f, 0}, 1.5f, 1, .22f, .6f};
}
pme_result YMPME_Pendulum_Init(pme_pendulum *p, const pme_pendulum_config *c)
{
	pme_pendulum v;
	float inertia;
	pme_quaternion q;
	if (!p || !c || !YMPME_VecFinite(c->anchor) || !isfinite(c->length) || c->length <= 0 ||
		!isfinite(c->mass) || c->mass <= 0 || !isfinite(c->radius) || c->radius <= 0 ||
		!isfinite(c->initialAngle) || fabsf(c->initialAngle) > 3.14159265358979323846f)
		return PME_INVALID_ARGUMENT;
	memset(&v, 0, sizeof(v));
	v.config = *c;
	inertia = .4f * c->mass * c->radius * c->radius;
	if (YMPME_Rigid_Init(&v.body, c->mass, YMPME_Vec3(inertia, inertia, inertia),
						 YMPME_VecAdd(c->anchor, YMPME_Vec3(0, -c->length, 0)),
						 YMPME_Quaternion_Identity()) != PME_OK)
		return PME_INVALID_ARGUMENT;
	if (YMPME_Joint_Init(&v.joint, PME_JOINT_REVOLUTE, &v.body, 1, PME_JOINT_WORLD, 0, c->anchor,
						 YMPME_Quaternion_Identity()) != PME_OK)
		return PME_INVALID_ARGUMENT;
	YMPME_Quaternion_AxisAngle(YMPME_Vec3(0, 0, 1), c->initialAngle, &q);
	v.body.orientation = q;
	v.body.position = YMPME_VecAdd(c->anchor, YMPME_Rotation_Apply(YMPME_Quaternion_Rotation(q),
																   YMPME_Vec3(0, -c->length, 0)));
	v.body.feedback.position = v.body.position;
	v.body.feedback.rotation = YMPME_Quaternion_Rotation(q);
	v.joint.coordinate = c->initialAngle;
	v.joint.wrappedAngle = c->initialAngle;
	v.joint.feedback.coordinate = c->initialAngle;
	*p = v;
	return PME_OK;
}
pme_result YMPME_Pendulum_Step(pme_pendulum *p, pme_vec3 gravity, float dt)
{
	pme_result r;
	if (!p)
		return PME_INVALID_ARGUMENT;
	r = YMPME_Joint_Step(&p->body, 1, &p->joint, 1, gravity, dt, 32, p->time + dt);
	if (r == PME_OK)
		p->time += dt;
	return r;
}
