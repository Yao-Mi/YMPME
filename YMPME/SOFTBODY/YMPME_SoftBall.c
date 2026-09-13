#include "YMPME_SoftBall.h"
static int valid(const pme_soft_ball *b)
{
	return b && isfinite(b->radius) && b->radius > 0 && isfinite(b->stiffness) &&
		   b->stiffness > 0 && isfinite(b->damping) && b->damping > 0 && isfinite(b->compression) &&
		   b->compression >= -.25f * b->radius && b->compression <= .55f * b->radius &&
		   isfinite(b->speed);
}
pme_result YMPME_SoftBall_Init(pme_soft_ball *b, float radius, float stiffness, float damping)
{
	pme_soft_ball fresh = {radius, stiffness, damping, 0, 0};
	if (!b || !valid(&fresh))
		return PME_INVALID_ARGUMENT;
	*b = fresh;
	return PME_OK;
}
pme_result YMPME_SoftBall_Step(pme_soft_ball *b, float force, float dt)
{
	if (!valid(b) || !isfinite(force) || !isfinite(dt) || dt <= 0 || dt > .02f)
		return PME_INVALID_ARGUMENT;
	float q = (b->damping * b->compression + dt * force) / (b->damping + dt * b->stiffness);
	if (!isfinite(q))
		return PME_INVALID_ARGUMENT;
	q = YMPME_Clamp(q, -.25f * b->radius, .55f * b->radius);
	float speed = (q - b->compression) / dt;
	if (!isfinite(speed))
		return PME_INVALID_ARGUMENT;
	b->compression = q;
	b->speed = speed;
	return PME_OK;
}
pme_vec3 YMPME_SoftBall_Scale(const pme_soft_ball *b)
{
	float x = 1 - b->compression / b->radius, y = 1 / sqrtf(x);
	return YMPME_Vec3(x, y, y);
}
pme_vec3 YMPME_SoftBall_Mode(const pme_soft_ball *b, pme_vec3 p)
{
	float inv = 1 / (b->radius - b->compression);
	return YMPME_Vec3(-p.x * inv, .5f * p.y * inv, .5f * p.z * inv);
}
