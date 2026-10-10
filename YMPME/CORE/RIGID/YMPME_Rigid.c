#include "YMPME_Rigid.h"
#include <string.h>
static pme_quaternion normalize(pme_quaternion q)
{
	float s = 1 / sqrtf(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
	return (pme_quaternion){q.x * s, q.y * s, q.z * s, q.w * s};
}
pme_quaternion YMPME_Quaternion_Identity(void) { return (pme_quaternion){0, 0, 0, 1}; }
pme_quaternion YMPME_Quaternion_Conjugate(pme_quaternion q)
{
	return (pme_quaternion){-q.x, -q.y, -q.z, q.w};
}
pme_quaternion YMPME_Quaternion_Multiply(pme_quaternion a, pme_quaternion b)
{
	return (pme_quaternion){a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
							a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
							a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
							a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z};
}
pme_result YMPME_Quaternion_AxisAngle(pme_vec3 axis, float angle, pme_quaternion *out)
{
	float n = YMPME_VecDot(axis, axis), s;
	if (!out || !YMPME_VecFinite(axis) || !isfinite(n) || n < 1e-12f || !isfinite(angle))
		return PME_INVALID_ARGUMENT;
	s = sinf(angle * .5f) / sqrtf(n);
	*out = (pme_quaternion){axis.x * s, axis.y * s, axis.z * s, cosf(angle * .5f)};
	return PME_OK;
}
pme_rotation YMPME_Quaternion_Rotation(pme_quaternion q)
{
	pme_rotation r;
	r.axis[0] = YMPME_Vec3(1 - 2 * (q.y * q.y + q.z * q.z), 2 * (q.x * q.y + q.z * q.w),
						   2 * (q.x * q.z - q.y * q.w));
	r.axis[1] = YMPME_Vec3(2 * (q.x * q.y - q.z * q.w), 1 - 2 * (q.x * q.x + q.z * q.z),
						   2 * (q.y * q.z + q.x * q.w));
	r.axis[2] = YMPME_Vec3(2 * (q.x * q.z + q.y * q.w), 2 * (q.y * q.z - q.x * q.w),
						   1 - 2 * (q.x * q.x + q.y * q.y));
	return r;
}
pme_vec3 YMPME_Quaternion_Log(pme_quaternion q)
{
	float s = sqrtf(q.x * q.x + q.y * q.y + q.z * q.z), f;
	if (q.w < 0)
	{
		q.x = -q.x;
		q.y = -q.y;
		q.z = -q.z;
		q.w = -q.w;
	}
	f = s > 1e-7f ? 2 * atan2f(s, q.w) / s : 2;
	return YMPME_Vec3(q.x * f, q.y * f, q.z * f);
}
pme_vec3 YMPME_Rotation_InverseApply(pme_rotation r, pme_vec3 v)
{
	return YMPME_Vec3(YMPME_VecDot(r.axis[0], v), YMPME_VecDot(r.axis[1], v),
					  YMPME_VecDot(r.axis[2], v));
}
pme_vec3 YMPME_Rigid_BoxInertia(float mass, pme_vec3 s)
{
	return YMPME_Vec3(mass * (s.y * s.y + s.z * s.z) / 12, mass * (s.x * s.x + s.z * s.z) / 12,
					  mass * (s.x * s.x + s.y * s.y) / 12);
}
int YMPME_Quaternion_IsValid(pme_quaternion q)
{
	float n = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
	return isfinite(n) && fabsf(n - 1) < 1e-3f;
}
pme_result YMPME_Rigid_Init(pme_rigid_body *b, float mass, pme_vec3 inertia, pme_vec3 pos,
							pme_quaternion q)
{
	pme_rigid_body v;
	if (!b || !isfinite(mass) || mass < 0 || !YMPME_VecFinite(inertia) || !YMPME_VecFinite(pos) ||
		!YMPME_Quaternion_IsValid(q) ||
		(mass > 0 && (mass < 1e-8f || inertia.x < 1e-8f || inertia.y < 1e-8f || inertia.z < 1e-8f)))
		return PME_INVALID_ARGUMENT;
	memset(&v, 0, sizeof(v));
	v.position = pos;
	v.orientation = normalize(q);
	v.mass = mass;
	v.inertia = inertia;
	if (mass > 0)
	{
		v.inverseMass = 1 / mass;
		v.inverseInertia = YMPME_Vec3(1 / inertia.x, 1 / inertia.y, 1 / inertia.z);
	}
	v.feedback.position = pos;
	v.feedback.rotation = YMPME_Quaternion_Rotation(v.orientation);
	v.feedback.valid = PME_FEEDBACK_POSE | PME_FEEDBACK_VELOCITY | PME_FEEDBACK_ANGULAR_VELOCITY;
	*b = v;
	return PME_OK;
}
int YMPME_Rigid_IsValid(const pme_rigid_body *b)
{
	return b && isfinite(b->mass) && b->mass >= 0 && isfinite(b->inverseMass) &&
		   YMPME_VecFinite(b->inertia) && YMPME_VecFinite(b->inverseInertia) &&
		   ((b->mass == 0 && b->inverseMass == 0 &&
			 YMPME_VecDot(b->inverseInertia, b->inverseInertia) == 0 &&
			 YMPME_VecDot(b->velocity, b->velocity) == 0 &&
			 YMPME_VecDot(b->angularVelocity, b->angularVelocity) == 0) ||
			(b->mass > 0 && b->inverseMass > 0 && b->inertia.x > 0 && b->inertia.y > 0 &&
			 b->inertia.z > 0 && b->inverseInertia.x > 0 && b->inverseInertia.y > 0 &&
			 b->inverseInertia.z > 0)) &&
		   YMPME_VecFinite(b->position) && YMPME_Quaternion_IsValid(b->orientation) &&
		   YMPME_VecFinite(b->velocity) && YMPME_VecFinite(b->angularVelocity) &&
		   YMPME_VecFinite(b->load.force) && YMPME_VecFinite(b->load.torque) &&
		   isfinite(b->feedback.time);
}
pme_vec3 YMPME_Rigid_InverseInertia(const pme_rigid_body *b, pme_vec3 v)
{
	pme_rotation r;
	pme_vec3 local;
	if (!b || !b->inverseMass)
		return YMPME_Vec3(0, 0, 0);
	r = YMPME_Quaternion_Rotation(b->orientation);
	local = YMPME_Rotation_InverseApply(r, v);
	local.x *= b->inverseInertia.x;
	local.y *= b->inverseInertia.y;
	local.z *= b->inverseInertia.z;
	return YMPME_Rotation_Apply(r, local);
}
pme_result YMPME_Rigid_AddForceAt(pme_rigid_body *b, pme_vec3 f, pme_vec3 p)
{
	pme_wrench w;
	if (!YMPME_Rigid_IsValid(b) || !YMPME_VecFinite(f) || !YMPME_VecFinite(p))
		return PME_INVALID_ARGUMENT;
	w.force = YMPME_VecAdd(b->load.force, f);
	w.torque = YMPME_VecAdd(b->load.torque, YMPME_VecCross(YMPME_VecSub(p, b->position), f));
	if (!YMPME_VecFinite(w.force) || !YMPME_VecFinite(w.torque))
		return PME_INVALID_ARGUMENT;
	b->load = w;
	return PME_OK;
}
pme_result YMPME_Rigid_AddTorque(pme_rigid_body *b, pme_vec3 t)
{
	if (!YMPME_Rigid_IsValid(b) || !YMPME_VecFinite(t) ||
		!YMPME_VecFinite(YMPME_VecAdd(b->load.torque, t)))
		return PME_INVALID_ARGUMENT;
	b->load.torque = YMPME_VecAdd(b->load.torque, t);
	return PME_OK;
}
void YMPME_Rigid_Impulse(pme_rigid_body *b, pme_vec3 p, pme_vec3 l)
{
	if (!b || !b->inverseMass)
		return;
	b->velocity = YMPME_VecAdd(b->velocity, YMPME_VecScale(p, b->inverseMass));
	b->angularVelocity = YMPME_VecAdd(b->angularVelocity, YMPME_Rigid_InverseInertia(b, l));
}
void YMPME_Rigid_Begin(pme_rigid_body *b, pme_vec3 gravity, float dt)
{
	pme_rotation r;
	pme_vec3 local, momentum, effectiveTorque;
	b->previousVelocity = b->velocity;
	b->previousAngularVelocity = b->angularVelocity;
	if (!b->inverseMass)
	{
		b->velocity = b->angularVelocity = YMPME_Vec3(0, 0, 0);
		return;
	}
	r = YMPME_Quaternion_Rotation(b->orientation);
	local = YMPME_Rotation_InverseApply(r, b->angularVelocity);
	momentum = YMPME_Rotation_Apply(
		r, YMPME_Vec3(local.x * b->inertia.x, local.y * b->inertia.y, local.z * b->inertia.z));
	effectiveTorque = YMPME_VecSub(b->load.torque, YMPME_VecCross(b->angularVelocity, momentum));
	YMPME_Rigid_Impulse(
		b, YMPME_VecScale(YMPME_VecAdd(b->load.force, YMPME_VecScale(gravity, b->mass)), dt),
		YMPME_VecScale(effectiveTorque, dt));
}
void YMPME_Rigid_End(pme_rigid_body *b, float dt, double time)
{
	if (b->inverseMass)
	{
		float speed = sqrtf(YMPME_VecDot(b->angularVelocity, b->angularVelocity));
		b->position = YMPME_VecAdd(b->position, YMPME_VecScale(b->velocity, dt));
		if (speed > 0)
		{
			float f = speed > 1e-6f ? sinf(speed * dt * .5f) / speed : dt * .5f;
			pme_quaternion dq = {b->angularVelocity.x * f, b->angularVelocity.y * f,
								 b->angularVelocity.z * f, cosf(speed * dt * .5f)};
			b->orientation = normalize(YMPME_Quaternion_Multiply(dq, b->orientation));
		}
	}
	b->feedback.position = b->position;
	b->feedback.rotation = YMPME_Quaternion_Rotation(b->orientation);
	b->feedback.velocity = b->velocity;
	b->feedback.angularVelocity = b->angularVelocity;
	b->feedback.acceleration =
		YMPME_VecScale(YMPME_VecSub(b->velocity, b->previousVelocity), 1 / dt);
	b->feedback.angularAcceleration =
		YMPME_VecScale(YMPME_VecSub(b->angularVelocity, b->previousAngularVelocity), 1 / dt);
	b->feedback.time = time;
	b->feedback.interval = dt;
	b->feedback.valid = 31;
	memset(&b->load, 0, sizeof(b->load));
}
pme_result YMPME_Rigid_Step(pme_rigid_body *b, pme_vec3 g, float dt, double time)
{
	if (!YMPME_Rigid_IsValid(b) || !YMPME_VecFinite(g) || !isfinite(dt) || dt <= 0 ||
		!isfinite(1.f / dt) || dt > .05f || !isfinite(time) || time <= b->feedback.time ||
		time < b->feedback.time + dt * .5)
		return PME_INVALID_ARGUMENT;
	/* Commit only finite results: an extreme finite load may overflow
	 * intermediates. */
	pme_rigid_body next = *b;
	YMPME_Rigid_Begin(&next, g, dt);
	YMPME_Rigid_End(&next, dt, time);
	if (!YMPME_Rigid_IsValid(&next) || !YMPME_VecFinite(next.feedback.acceleration) ||
		!YMPME_VecFinite(next.feedback.angularAcceleration))
		return PME_INVALID_ARGUMENT;
	*b = next;
	return PME_OK;
}
