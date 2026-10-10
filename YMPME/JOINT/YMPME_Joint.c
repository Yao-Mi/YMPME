#include "YMPME_Joint.h"
#include <float.h>
#include <string.h>
#define PI 3.14159265358979323846f
#define V0 YMPME_Vec3(0, 0, 0)
static pme_vec3 add(pme_vec3 a, pme_vec3 b) { return YMPME_VecAdd(a, b); }
static pme_vec3 sub(pme_vec3 a, pme_vec3 b) { return YMPME_VecSub(a, b); }
static pme_vec3 scale(pme_vec3 a, float f) { return YMPME_VecScale(a, f); }
static pme_vec3 cross(pme_vec3 a, pme_vec3 b) { return YMPME_VecCross(a, b); }
static float dot(pme_vec3 a, pme_vec3 b) { return YMPME_VecDot(a, b); }
static float length(pme_vec3 a) { return sqrtf(dot(a, a)); }
static float wrap(float a)
{
	while (a > PI)
		a -= 2 * PI;
	while (a <= -PI)
		a += 2 * PI;
	return a;
}
static pme_quaternion frame(const pme_rigid_body *b, pme_quaternion local)
{
	return b ? YMPME_Quaternion_Multiply(b->orientation, local) : local;
}
static pme_vec3 offset(const pme_rigid_body *b, pme_vec3 local)
{
	return b ? YMPME_Rotation_Apply(YMPME_Quaternion_Rotation(b->orientation), local) : local;
}
static pme_vec3 point(const pme_rigid_body *b, pme_vec3 local)
{
	return add(b ? b->position : V0, offset(b, local));
}
static int motorValid(pme_joint_motor m)
{
	return m.mode >= PME_JOINT_MOTOR_OFF && m.mode <= PME_JOINT_MOTOR_EFFORT && isfinite(m.speed) &&
		   isfinite(m.effort) && isfinite(m.maxEffort) && m.maxEffort >= 0;
}
static int limitValid(pme_joint_limit l)
{
	return isfinite(l.lower) && isfinite(l.upper) && l.lower <= l.upper;
}
pme_result YMPME_Joint_SetMotor(pme_joint *j, pme_joint_motor m)
{
	if (!j || j->type == PME_JOINT_FIXED || !motorValid(m))
		return PME_INVALID_ARGUMENT;
	j->motor = m;
	return PME_OK;
}
pme_result YMPME_Joint_SetLimit(pme_joint *j, pme_joint_limit l)
{
	if (!j || j->type == PME_JOINT_FIXED || !limitValid(l))
		return PME_INVALID_ARGUMENT;
	j->limit = l;
	return PME_OK;
}
pme_result YMPME_Joint_Init(pme_joint *j, pme_joint_type type, const pme_rigid_body *b,
							size_t count, size_t a, size_t c, pme_vec3 anchor,
							pme_quaternion orientation)
{
	pme_joint v;
	pme_quaternion inverse;
	if (!j || !b || type < PME_JOINT_FIXED || type > PME_JOINT_PRISMATIC || c >= count ||
		(a != PME_JOINT_WORLD && (a >= count || a == c || !YMPME_Rigid_IsValid(&b[a]))) ||
		!YMPME_Rigid_IsValid(&b[c]) || !YMPME_VecFinite(anchor) ||
		!YMPME_Rotation_IsValid(YMPME_Quaternion_Rotation(orientation)) ||
		fabsf(orientation.x * orientation.x + orientation.y * orientation.y +
			  orientation.z * orientation.z + orientation.w * orientation.w - 1) > 1e-3f)
		return PME_INVALID_ARGUMENT;
	memset(&v, 0, sizeof(v));
	v.type = type;
	v.bodyA = a;
	v.bodyB = c;
	v.enabled = 1;
	v.localAnchorA = a == PME_JOINT_WORLD
						 ? anchor
						 : YMPME_Rotation_InverseApply(YMPME_Quaternion_Rotation(b[a].orientation),
													   sub(anchor, b[a].position));
	v.localAnchorB = YMPME_Rotation_InverseApply(YMPME_Quaternion_Rotation(b[c].orientation),
												 sub(anchor, b[c].position));
	inverse = a == PME_JOINT_WORLD ? YMPME_Quaternion_Identity()
								   : YMPME_Quaternion_Conjugate(b[a].orientation);
	v.localFrameA = YMPME_Quaternion_Multiply(inverse, orientation);
	v.localFrameB =
		YMPME_Quaternion_Multiply(YMPME_Quaternion_Conjugate(b[c].orientation), orientation);
	*j = v;
	return PME_OK;
}
typedef struct geometry_
{
	pme_vec3 ra, rb, d, angularError;
	pme_rotation axes;
	float wrapped, coordinate, speed;
} geometry;
static geometry measure(pme_joint *j, const pme_rigid_body *a, const pme_rigid_body *b)
{
	geometry g;
	pme_quaternion qa = frame(a, j->localFrameA), qb = frame(b, j->localFrameB);
	pme_quaternion rel = YMPME_Quaternion_Multiply(YMPME_Quaternion_Conjugate(qa), qb);
	pme_rotation br = YMPME_Quaternion_Rotation(qb);
	g.axes = YMPME_Quaternion_Rotation(qa);
	g.ra = offset(a, j->localAnchorA);
	g.rb = offset(b, j->localAnchorB);
	g.d = sub(point(b, j->localAnchorB), point(a, j->localAnchorA));
	g.wrapped = wrap(2 * atan2f(rel.z, rel.w));
	g.coordinate = j->coordinate;
	g.speed = 0;
	g.angularError = YMPME_Rotation_Apply(g.axes, YMPME_Quaternion_Log(rel));
	if (j->type == PME_JOINT_REVOLUTE)
	{
		pme_vec3 cr = cross(g.axes.axis[2], br.axis[2]);
		float n = length(cr), d = YMPME_Clamp(dot(g.axes.axis[2], br.axis[2]), -1, 1);
		g.angularError =
			n > 1e-7f ? scale(cr, atan2f(n, d) / n) : (d < 0 ? scale(g.axes.axis[0], PI) : V0);
		g.coordinate += wrap(g.wrapped - j->wrappedAngle);
		g.speed = dot(sub(b->angularVelocity, a ? a->angularVelocity : V0), g.axes.axis[2]);
	}
	else if (j->type == PME_JOINT_PRISMATIC)
	{
		pme_vec3 va = a ? add(a->velocity, cross(a->angularVelocity, add(g.ra, g.d))) : V0;
		g.coordinate = dot(g.d, g.axes.axis[2]);
		g.speed = dot(sub(add(b->velocity, cross(b->angularVelocity, g.rb)), va), g.axes.axis[2]);
	}
	return g;
}
static void row(pme_joint *j, const pme_rigid_body *a, const pme_rigid_body *b, pme_vec3 la,
				pme_vec3 aa, pme_vec3 lb, pme_vec3 ab, float target, float lower, float upper)
{
	pme_joint_row *r = &j->rows[j->rowCount++];
	float k;
	memset(r, 0, sizeof(*r));
	r->linearA = la;
	r->angularA = aa;
	r->linearB = lb;
	r->angularB = ab;
	r->target = target;
	r->lower = lower;
	r->upper = upper;
	r->responseLinearA = scale(la, a ? a->inverseMass : 0);
	r->responseLinearB = scale(lb, b->inverseMass);
	r->responseAngularA = YMPME_Rigid_InverseInertia(a, aa);
	r->responseAngularB = YMPME_Rigid_InverseInertia(b, ab);
	k = dot(la, r->responseLinearA) + dot(lb, r->responseLinearB) + dot(aa, r->responseAngularA) +
		dot(ab, r->responseAngularB);
	r->inverseEffectiveMass = k > 1e-12f ? 1 / k : 0;
}
static void linear(pme_joint *j, const pme_rigid_body *a, const pme_rigid_body *b, geometry g,
				   pme_vec3 axis, float target, float lo, float hi)
{
	pme_vec3 armA = j->type == PME_JOINT_PRISMATIC ? add(g.ra, g.d) : g.ra;
	row(j, a, b, scale(axis, -1), scale(cross(armA, axis), -1), axis, cross(g.rb, axis), target, lo,
		hi);
}
static void angular(pme_joint *j, const pme_rigid_body *a, const pme_rigid_body *b, pme_vec3 axis,
					float target, float lo, float hi)
{
	row(j, a, b, V0, scale(axis, -1), V0, axis, target, lo, hi);
}
static void dof(pme_joint *j, const pme_rigid_body *a, const pme_rigid_body *b, geometry g,
				float target, float lo, float hi)
{
	if (j->type == PME_JOINT_REVOLUTE)
		angular(j, a, b, g.axes.axis[2], target, lo, hi);
	else
		linear(j, a, b, g, g.axes.axis[2], target, lo, hi);
}
static void prepare(pme_joint *j, const pme_rigid_body *a, const pme_rigid_body *b, float dt)
{
	geometry g = measure(j, a, b);
	unsigned i;
	float lo = 0, hi = 0, target = 0;
	j->coordinate = g.coordinate;
	j->wrappedAngle = g.wrapped;
	j->rowCount = 0;
	memset(&j->feedback, 0, sizeof(j->feedback));
	if (!j->enabled)
		return;
	for (i = 0; i < (j->type == PME_JOINT_PRISMATIC ? 2u : 3u); ++i)
		linear(j, a, b, g, g.axes.axis[i], -.2f * dot(g.d, g.axes.axis[i]) / dt, -FLT_MAX, FLT_MAX);
	for (i = 0; i < (j->type == PME_JOINT_REVOLUTE ? 2u : 3u); ++i)
		angular(j, a, b, g.axes.axis[i], -.2f * dot(g.angularError, g.axes.axis[i]) / dt, -FLT_MAX,
				FLT_MAX);
	if (j->type == PME_JOINT_FIXED)
		return;
	if (j->motor.mode == PME_JOINT_MOTOR_SPEED)
	{
		hi = j->motor.maxEffort * dt;
		lo = -hi;
		target = j->motor.speed;
	}
	else if (j->motor.mode == PME_JOINT_MOTOR_EFFORT)
	{
		lo = hi = YMPME_Clamp(j->motor.effort, -j->motor.maxEffort, j->motor.maxEffort) * dt;
	}
	dof(j, a, b, g, target, lo, hi);
	if (j->limit.enabled)
	{
		float error = j->limit.lower - g.coordinate;
		dof(j, a, b, g, error * (error > 0 ? .2f : 1) / dt, 0, FLT_MAX);
		error = j->limit.upper - g.coordinate;
		dof(j, a, b, g, error * (error < 0 ? .2f : 1) / dt, -FLT_MAX, 0);
	}
}
static void solve(pme_joint *j, pme_rigid_body *a, pme_rigid_body *b)
{
	unsigned i;
	for (i = 0; i < j->rowCount; ++i)
	{
		pme_joint_row *r = &j->rows[i];
		float speed, lambda, next;
		if (!r->inverseEffectiveMass)
			continue;
		speed = dot(r->linearB, b->velocity) + dot(r->angularB, b->angularVelocity);
		if (a)
			speed += dot(r->linearA, a->velocity) + dot(r->angularA, a->angularVelocity);
		next = YMPME_Clamp(r->impulse + (r->target - speed) * r->inverseEffectiveMass, r->lower,
						   r->upper);
		lambda = next - r->impulse;
		r->impulse = next;
		if (a)
		{
			a->velocity = add(a->velocity, scale(r->responseLinearA, lambda));
			a->angularVelocity = add(a->angularVelocity, scale(r->responseAngularA, lambda));
		}
		b->velocity = add(b->velocity, scale(r->responseLinearB, lambda));
		b->angularVelocity = add(b->angularVelocity, scale(r->responseAngularB, lambda));
	}
}
static void feedback(pme_joint *j, const pme_rigid_body *a, const pme_rigid_body *b, float dt,
					 double time)
{
	geometry g = measure(j, a, b);
	unsigned i;
	j->coordinate = g.coordinate;
	j->wrappedAngle = g.wrapped;
	j->feedback.coordinate = g.coordinate;
	j->feedback.speed = g.speed;
	j->feedback.time = time;
	j->feedback.interval = dt;
	j->feedback.valid = 1;
	j->feedback.anchorError = length(
		j->type == PME_JOINT_PRISMATIC ? sub(g.d, scale(g.axes.axis[2], g.coordinate)) : g.d);
	j->feedback.angularError = length(g.angularError);
	for (i = 0; i < j->rowCount; ++i)
	{
		pme_joint_row *r = &j->rows[i];
		float f = r->impulse / dt;
		j->feedback.reactionA.force = add(j->feedback.reactionA.force, scale(r->linearA, f));
		j->feedback.reactionA.torque = add(j->feedback.reactionA.torque, scale(r->angularA, f));
		j->feedback.reactionB.force = add(j->feedback.reactionB.force, scale(r->linearB, f));
		j->feedback.reactionB.torque = add(j->feedback.reactionB.torque, scale(r->angularB, f));
	}
	if (j->enabled && j->type != PME_JOINT_FIXED)
		j->feedback.motorEffort = j->rows[5].impulse / dt;
}
static int jointValid(const pme_joint *j, size_t n)
{
	return j->type >= PME_JOINT_FIXED && j->type <= PME_JOINT_PRISMATIC && j->bodyB < n &&
		   (j->bodyA == PME_JOINT_WORLD || (j->bodyA < n && j->bodyA != j->bodyB)) &&
		   YMPME_VecFinite(j->localAnchorA) && YMPME_VecFinite(j->localAnchorB) &&
		   YMPME_Quaternion_IsValid(j->localFrameA) && YMPME_Quaternion_IsValid(j->localFrameB) &&
		   isfinite(j->coordinate) && isfinite(j->wrappedAngle) && fabsf(j->wrappedAngle) <= PI &&
		   motorValid(j->motor) && limitValid(j->limit);
}
pme_result YMPME_Joint_ValidateStep(const pme_rigid_body *b, size_t n, const pme_joint *j,
									size_t count, pme_vec3 gravity, float dt, unsigned iterations,
									double time)
{
	size_t i;
	if ((n && !b) || (count && !j) || !isfinite(dt) || dt <= 0 || !isfinite(1.f / dt) ||
		dt > .05f || !iterations || iterations > 256 || !YMPME_VecFinite(gravity) ||
		!isfinite(time))
		return PME_INVALID_ARGUMENT;
	for (i = 0; i < n; ++i)
		if (!YMPME_Rigid_IsValid(&b[i]) || time <= b[i].feedback.time ||
			time < b[i].feedback.time + dt * .5)
			return PME_INVALID_ARGUMENT;
	for (i = 0; i < count; ++i)
		if (!jointValid(&j[i], n))
			return PME_INVALID_ARGUMENT;
	return PME_OK;
}

void YMPME_Joint_Prepare(pme_rigid_body *b, pme_joint *j, size_t count, float dt)
{
	for (size_t i = 0; i < count; ++i)
		prepare(&j[i], j[i].bodyA == PME_JOINT_WORLD ? NULL : &b[j[i].bodyA], &b[j[i].bodyB], dt);
}
void YMPME_Joint_SolveIteration(pme_rigid_body *b, pme_joint *j, size_t count)
{
	for (size_t i = 0; i < count; ++i)
		solve(&j[i], j[i].bodyA == PME_JOINT_WORLD ? NULL : &b[j[i].bodyA], &b[j[i].bodyB]);
}
void YMPME_Joint_Finish(const pme_rigid_body *b, pme_joint *j, size_t count, float dt, double time)
{
	for (size_t i = 0; i < count; ++i)
		feedback(&j[i], j[i].bodyA == PME_JOINT_WORLD ? NULL : &b[j[i].bodyA], &b[j[i].bodyB], dt,
				 time);
}
pme_result YMPME_Joint_Step(pme_rigid_body *b, size_t n, pme_joint *j, size_t count,
							pme_vec3 gravity, float dt, unsigned iterations, double time)
{
	pme_result result = YMPME_Joint_ValidateStep(b, n, j, count, gravity, dt, iterations, time);
	if (result != PME_OK)
		return result;
	for (size_t i = 0; i < n; ++i)
		YMPME_Rigid_Begin(&b[i], gravity, dt);
	YMPME_Joint_Prepare(b, j, count, dt);
	for (unsigned k = 0; k < iterations; ++k)
		YMPME_Joint_SolveIteration(b, j, count);
	for (size_t i = 0; i < n; ++i)
		YMPME_Rigid_End(&b[i], dt, time);
	YMPME_Joint_Finish(b, j, count, dt, time);
	return PME_OK;
}
