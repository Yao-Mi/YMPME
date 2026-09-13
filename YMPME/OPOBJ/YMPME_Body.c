#include "YMPME_Body.h"
#include "../CORE/YMPME_MathBase.h"
#include <string.h>
pme_body_desc YMPME_Body_DefaultDesc(void)
{
	pme_body_desc d;
	memset(&d, 0, sizeof(d));
	d.type = PME_DYNAMIC;
	d.rotation = YMPME_Rotation_Identity();
	d.shape = PME_SPHERE;
	d.halfExtents = YMPME_Vec3(0.5f, 0.5f, 0.5f);
	d.radius = 0.5f;
	d.mass = 1.0f;
	d.friction = 0.5f;
	d.category = 1u;
	d.mask = UINT32_MAX;
	return d;
}
pme_result YMPME_Body_Init(PME_Body body, const pme_body_desc* d)
{
	pme_body value;
	if (!body || !d || d->type < PME_STATIC || d->type > PME_KINEMATIC ||
		d->shape < PME_SPHERE || d->shape > PME_CONVEX_MESH || d->collisionGroup >= 32 ||
		!YMPME_VecFinite(d->position) || !YMPME_VecFinite(d->velocity) ||
		!isfinite(d->restitution) || d->restitution < 0 || d->restitution > 1 ||
		!isfinite(d->friction) || d->friction < 0 ||
		!isfinite(d->linearDamping) || d->linearDamping < 0)
		return PME_INVALID_ARGUMENT;
	if (d->type == PME_DYNAMIC && (!isfinite(d->mass) || d->mass <= 0 || !isfinite(1.0f/d->mass)))
		return PME_INVALID_ARGUMENT;
	if (d->shape == PME_SPHERE && (!isfinite(d->radius) || d->radius <= 0))
		return PME_INVALID_ARGUMENT;
	if (d->shape == PME_BOX && (!YMPME_VecFinite(d->halfExtents) ||
		d->halfExtents.x <= 0 || d->halfExtents.y <= 0 || d->halfExtents.z <= 0))
		return PME_INVALID_ARGUMENT;
	if (d->shape == PME_CONVEX_MESH && (!d->mesh || !YMPME_ConvexMesh_HasVertices(d->mesh) || !YMPME_ConvexMesh_HasIndices(d->mesh) ||
		!d->mesh->faceAxes || !d->mesh->edgeAxes || d->mesh->vertexCount < 4 ||
		d->mesh->faceCount == 0 || d->mesh->edgeCount == 0 || !YMPME_Rotation_IsValid(d->rotation)))
		return PME_INVALID_ARGUMENT;
	memset(&value, 0, sizeof(value));
	value.state = *d;
	value.previousPosition = d->position;
	value.inverseMass = d->type == PME_DYNAMIC ? 1.0f/d->mass : 0;
	value.active = 1;
	*body = value;
	return PME_OK;
}
pme_result YMPME_Body_AddForce(PME_Body body, pme_vec3 force)
{
	if (!body || !body->active || !YMPME_VecFinite(force)) return PME_INVALID_ARGUMENT;
	if (body->state.type == PME_DYNAMIC)
	{
		pme_vec3 sum = YMPME_VecAdd(body->force, force);
		if (!YMPME_VecFinite(sum)) return PME_INVALID_ARGUMENT;
		body->force = sum;
	}
	return PME_OK;
}
pme_result YMPME_Body_AddImpulse(PME_Body body, pme_vec3 impulse)
{
	if (!body || !body->active || !YMPME_VecFinite(impulse)) return PME_INVALID_ARGUMENT;
	if (body->state.type == PME_DYNAMIC)
	{
		pme_vec3 v = YMPME_VecAdd(body->state.velocity, YMPME_VecScale(impulse, body->inverseMass));
		if (!YMPME_VecFinite(v)) return PME_INVALID_ARGUMENT;
		body->state.velocity = v;
	}
	return PME_OK;
}
pme_result YMPME_Body_SetPosition(PME_Body body, pme_vec3 p)
{
	if (!body || !body->active || !YMPME_VecFinite(p)) return PME_INVALID_ARGUMENT;
	body->state.position = body->previousPosition = p;
	return PME_OK;
}
pme_vec3 YMPME_Body_InterpolatedPosition(const pme_body* body, float alpha)
{
	if (!body) return YMPME_Vec3(0, 0, 0);
	if (!isfinite(alpha)) alpha = 1;
	alpha = YMPME_Clamp(alpha, 0, 1);
	return YMPME_VecAdd(body->previousPosition,
		YMPME_VecScale(YMPME_VecSub(body->state.position, body->previousPosition), alpha));
}

pme_result YMPME_Body_SetRotation(PME_Body body, pme_rotation rotation)
{
	if (!body || !body->active || !YMPME_Rotation_IsValid(rotation)) return PME_INVALID_ARGUMENT;
	body->state.rotation = rotation;
	return PME_OK;
}
