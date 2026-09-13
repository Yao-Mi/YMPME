#include "YMPME_SoftPad.h"
#include "../CORE/YMPME_MathBase.h"
#include <string.h>
#define V YMPME_Vec3
#define add YMPME_VecAdd
#define sub YMPME_VecSub
#define mul YMPME_VecScale
#define dot YMPME_VecDot
#define cross YMPME_VecCross
static float norm(pme_vec3 p) { return sqrtf(dot(p, p)); }
pme_soft_pad_desc YMPME_SoftPad_Default(void)
{
	return (pme_soft_pad_desc){80000, 220, 100000, 180, .85f, .02f};
}
static pme_vec3 rotate(pme_vec3 v, pme_vec3 omega, float dt)
{
	float speed = norm(omega);
	if (speed < 1e-8f)
		return v;
	pme_vec3 axis = mul(omega, 1 / speed);
	float a = speed * dt;
	return add(add(mul(v, cosf(a)), mul(cross(axis, v), sinf(a))),
			   mul(axis, dot(axis, v) * (1 - cosf(a))));
}
static pme_soft_pad_node one(const pme_soft_pad_desc *d, const pme_surface_patch *p,
							 const pme_soft_pad_sample *sample, const pme_soft_pad_frame *frame,
							 float dt, pme_soft_pad_node old)
{
	pme_soft_pad_node n = {0};
	float freeCompression = old.compression * d->damping / (d->damping + dt * d->stiffness);
	float required = sample && sample->hit ? fminf(sample->compression, d->maxCompression) : 0;
	n.compression = fmaxf(freeCompression, required);
	n.compressionSpeed = (n.compression - old.compression) / dt;
	n.position = sub(p->position, mul(p->normal, n.compression));
	if (!sample || !sample->hit || required <= 0 || required < freeCompression)
		return n;
	float cosine = -dot(sample->normal, p->normal);
	float axisForce =
		p->area * fmaxf(0, d->stiffness * n.compression + d->damping * n.compressionSpeed);
	float normalForce = axisForce / cosine;
	n.contact = normalForce > 0;
	if (!n.contact)
		return n;
	if (d->friction == 0)
	{
		n.force = mul(sample->normal, normalForce);
		return n;
	}
	pme_vec3 padVelocity =
		sub(add(p->velocity, cross(frame->supportAngularVelocity, sub(n.position, p->position))),
			mul(p->normal, n.compressionSpeed));
	pme_vec3 relative = sub(sample->velocity, padVelocity);
	pme_vec3 tangent = sub(relative, mul(sample->normal, dot(relative, sample->normal)));
	n.shear = rotate(old.shear, frame->supportAngularVelocity, dt);
	n.shear = sub(n.shear, mul(sample->normal, dot(n.shear, sample->normal)));
	n.shear = add(n.shear, mul(tangent, dt));
	pme_vec3 shearForce =
		mul(add(mul(n.shear, d->shearStiffness), mul(tangent, d->shearDamping)), p->area);
	float mag = norm(shearForce), limit = d->friction * normalForce;
	if (mag > limit && mag > 0)
	{
		shearForce = mul(shearForce, limit / mag);
		n.shear = mul(sub(mul(shearForce, 1 / p->area), mul(tangent, d->shearDamping)),
					  1 / d->shearStiffness);
	}
	n.force = add(mul(sample->normal, normalForce), shearForce);
	return n;
}
pme_result YMPME_SoftPad_Step(const pme_soft_pad_desc *d, const pme_surface_patch *rest,
							  const pme_soft_pad_sample *samples, size_t count,
							  const pme_soft_pad_frame *frame, float dt, pme_soft_pad_node *nodes,
							  size_t capacity, pme_soft_pad_result *out)
{
	if (!d || !frame || !out || (count && (!rest || !nodes)) || !isfinite(dt) || dt <= 0 ||
		dt > .02f || !isfinite(d->stiffness) || d->stiffness <= 0 || !isfinite(d->damping) ||
		d->damping < 0 || !isfinite(d->shearStiffness) || d->shearStiffness <= 0 ||
		!isfinite(d->shearDamping) || d->shearDamping < 0 || !isfinite(d->friction) ||
		d->friction < 0 || !isfinite(d->maxCompression) || d->maxCompression <= 0 ||
		!YMPME_VecFinite(frame->supportOrigin) || !YMPME_VecFinite(frame->otherOrigin) ||
		!YMPME_VecFinite(frame->supportAngularVelocity))
		return PME_INVALID_ARGUMENT;
	if (capacity < count)
		return PME_CAPACITY_EXCEEDED;
	pme_soft_pad_result result = {0};
	for (size_t i = 0; i < count; ++i)
	{
		const pme_surface_patch *p = &rest[i];
		const pme_soft_pad_sample *s = samples ? &samples[i] : NULL;
		if (!YMPME_VecFinite(p->position) || !YMPME_VecFinite(p->velocity) ||
			!YMPME_VecFinite(p->normal) || fabsf(dot(p->normal, p->normal) - 1) > 1e-4f ||
			!isfinite(p->area) || p->area <= 0 || !isfinite(nodes[i].compression) ||
			nodes[i].compression < 0 || nodes[i].compression > d->maxCompression ||
			!YMPME_VecFinite(nodes[i].shear))
			return PME_INVALID_ARGUMENT;
		if (s && s->hit &&
			(!isfinite(s->compression) || s->compression < 0 || !YMPME_VecFinite(s->normal) ||
			 fabsf(dot(s->normal, s->normal) - 1) > 1e-4f || -dot(s->normal, p->normal) < .15f ||
			 !YMPME_VecFinite(s->velocity)))
			return PME_INVALID_ARGUMENT;
		pme_soft_pad_node n = one(d, p, s, frame, dt, nodes[i]);
		if (!isfinite(n.compression) || !isfinite(n.compressionSpeed) ||
			!YMPME_VecFinite(n.position) || !YMPME_VecFinite(n.force) || !YMPME_VecFinite(n.shear))
			return PME_INVALID_ARGUMENT;
		result.support.force = add(result.support.force, n.force);
		result.support.torque =
			add(result.support.torque, cross(sub(n.position, frame->supportOrigin), n.force));
		result.other.force = sub(result.other.force, n.force);
		result.other.torque =
			sub(result.other.torque, cross(sub(n.position, frame->otherOrigin), n.force));
		result.contacts += (size_t)n.contact;
		result.bottomedOut += (size_t)(s && s->hit && s->compression > d->maxCompression);
		result.sampledArea += n.contact ? p->area : 0;
		result.maxCompression = fmaxf(result.maxCompression, n.compression);
		result.elasticEnergy += .5f * p->area *
								(d->stiffness * n.compression * n.compression +
								 d->shearStiffness * dot(n.shear, n.shear));
	}
	if (!YMPME_VecFinite(result.support.force) || !YMPME_VecFinite(result.support.torque) ||
		!YMPME_VecFinite(result.other.force) || !YMPME_VecFinite(result.other.torque) ||
		!isfinite(result.elasticEnergy) || !isfinite(result.sampledArea))
		return PME_INVALID_ARGUMENT;
	for (size_t i = 0; i < count; ++i)
		nodes[i] = one(d, &rest[i], samples ? &samples[i] : NULL, frame, dt, nodes[i]);
	*out = result;
	return PME_OK;
}
