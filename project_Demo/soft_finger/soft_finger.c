#include "soft_finger.h"
#include <string.h>
#include <float.h>
#define V YMPME_Vec3
#define add YMPME_VecAdd
#define sub YMPME_VecSub
#define mul YMPME_VecScale
#define dot YMPME_VecDot
#define cross YMPME_VecCross
static pme_vec3 unit(pme_vec3 p) { return mul(p, 1 / sqrtf(dot(p, p))); }
static pme_soft_pad_sample query(const soft_finger *s, pme_vec3 p, pme_vec3 n)
{
	const pme_rigid_body *b = &s->grasp.bodies[GRASP_OBJECT];
	pme_rotation r = YMPME_Quaternion_Rotation(b->orientation);
	pme_vec3 local = YMPME_Rotation_InverseApply(r, sub(p, b->position));
	pme_vec3 direction = YMPME_Rotation_InverseApply(r, n), normal = V(0, 0, 0);
	float compression = 0;
	if (s->grasp.shape == 0)
	{
		pme_vec3 scale = YMPME_SoftBall_Scale(&s->ball);
		pme_vec3 axes = mul(scale, s->ball.radius);
		pme_vec3 d = V(local.x / axes.x, local.y / axes.y, local.z / axes.z);
		pme_vec3 ray = V(direction.x / axes.x, direction.y / axes.y, direction.z / axes.z);
		float aa = dot(ray, ray), dn = dot(d, ray), disc = dn * dn - aa * (dot(d, d) - 1);
		if (disc <= 0)
			return (pme_soft_pad_sample){0};
		compression = (dn + sqrtf(disc)) / aa;
		if (compression <= 0)
			return (pme_soft_pad_sample){0};
		pme_vec3 point = sub(local, mul(direction, compression));
		normal =
			YMPME_Rotation_Apply(r, unit(V(point.x / (axes.x * axes.x), point.y / (axes.y * axes.y),
										   point.z / (axes.z * axes.z))));
	}
	else
	{
		/* Convex line clipping: p + t*n. Entry t<0 requires inward travel -t. */
		float lo = -FLT_MAX, hi = FLT_MAX;
		const grasp_geometry *g = &s->grasp.geometry;
		for (size_t k = 0; k < g->mesh.triangleCount; ++k)
		{
			pme_vec3 a = g->vertices[g->indices[3 * k]], bb = g->vertices[g->indices[3 * k + 1]],
					 c = g->vertices[g->indices[3 * k + 2]];
			pme_vec3 axis = unit(cross(sub(bb, a), sub(c, a)));
			if (dot(axis, a) < 0)
				axis = mul(axis, -1);
			float den = dot(axis, direction), dist = dot(axis, sub(a, local));
			if (fabsf(den) < 1e-7f)
			{
				if (dist < 0)
					return (pme_soft_pad_sample){0};
				continue;
			}
			float t = dist / den;
			if (den < 0)
			{
				if (t > lo)
				{
					lo = t;
					normal = YMPME_Rotation_Apply(r, axis);
				}
			}
			else
				hi = fminf(hi, t);
			if (lo > hi)
				return (pme_soft_pad_sample){0};
		}
		compression = -lo;
		if (compression <= 0 || !isfinite(compression))
			return (pme_soft_pad_sample){0};
	}
	if (-dot(normal, n) < .15f)
		return (pme_soft_pad_sample){0};
	pme_vec3 point = sub(p, mul(n, compression));
	pme_vec3 velocity = add(b->velocity, cross(b->angularVelocity, sub(point, b->position)));
	if (s->grasp.shape == 0)
	{
		pme_vec3 localPoint = YMPME_Rotation_InverseApply(r, sub(point, b->position));
		velocity = add(
			velocity,
			YMPME_Rotation_Apply(r, mul(YMPME_SoftBall_Mode(&s->ball, localPoint), s->ball.speed)));
	}
	return (pme_soft_pad_sample){1, compression, normal, velocity};
}
int SoftFinger_Init(soft_finger *s, int shape, int hard)
{
	if (!s)
		return 0;
	memset(s, 0, sizeof(*s));
	if (!Grasp_InitShape(&s->grasp, 0, shape))
		return 0;
	if (YMPME_SoftBall_Init(&s->ball, .22f, 40.f, 14.f) != PME_OK)
		return 0;
	s->material = YMPME_SoftPad_Default();
	s->hard = !!hard;
	if (hard)
	{
		s->material.stiffness *= 3;
		s->material.damping *= 1.5f;
	}
	return 1;
}
int SoftFinger_Step(soft_finger *s)
{
	grasp_lab *g = &s->grasp;
	if (g->shape == 0 && !SoftFinger_BallStep(s))
		return 0;
	for (int side = 0; side < 2; ++side)
	{
		pme_rigid_body *b = &g->bodies[side + 1];
		pme_rotation r = YMPME_Quaternion_Rotation(b->orientation);
		pme_vec3 n = YMPME_Rotation_Apply(r, V(side ? -1 : 1, 0, 0));
		for (int k = 0; k < GRASP_CELLS; ++k)
		{
			pme_vec3 local = V((side ? -1 : 1) * (.05f + s->material.maxCompression),
							   (.5f - (k / 8 + .5f) / 16) * .8f, ((k % 8 + .5f) / 8 - .5f) * .3f);
			pme_vec3 offset = YMPME_Rotation_Apply(r, local);
			s->rest[side][k] = (pme_surface_patch){
				add(b->position, offset), n, add(b->velocity, cross(b->angularVelocity, offset)),
				.8f * .3f / GRASP_CELLS};
			s->samples[side][k] = query(s, s->rest[side][k].position, n);
		}
		pme_soft_pad_frame frame = {b->position, b->angularVelocity,
									g->bodies[GRASP_OBJECT].position};
		s->material.friction = g->friction;
		if (YMPME_SoftPad_Step(&s->material, s->rest[side], s->samples[side], GRASP_CELLS, &frame,
							   GRASP_DT, s->nodes[side], GRASP_CELLS, &s->result[side]) != PME_OK)
			return 0;
	}
	for (int side = 0; side < 2; ++side)
	{
		pme_rigid_body *a = &g->bodies[side + 1], *b = &g->bodies[GRASP_OBJECT];
		pme_soft_pad_result *r = &s->result[side];
		if (YMPME_Rigid_AddForceAt(a, r->support.force, a->position) != PME_OK ||
			YMPME_Rigid_AddTorque(a, r->support.torque) != PME_OK ||
			YMPME_Rigid_AddForceAt(b, r->other.force, b->position) != PME_OK ||
			YMPME_Rigid_AddTorque(b, r->other.torque) != PME_OK)
			return 0;
	}
	if (!Grasp_Step(g))
		return 0;
	pme_sensor_stamp stamp = {g->world.time, g->world.time, g->steps, 1};
	for (int side = 0; side < 2; ++side)
	{
		for (int k = 0; k < GRASP_CELLS; ++k)
		{
			g->patches[side][k] = s->rest[side][k];
			g->patches[side][k].position = s->nodes[side][k].position;
			g->forces[side][k] = add(g->forces[side][k], s->nodes[side][k].force);
		}
		g->fingerForce[side] = add(g->fingerForce[side], s->result[side].support.force);
		g->gripForce = add(g->gripForce, s->result[side].other.force);
		g->objectContact.force = add(g->objectContact.force, s->result[side].other.force);
		g->objectContact.torque = add(g->objectContact.torque, s->result[side].other.torque);
		if (YMPME_SurfaceTactile_Measure(g->patches[side], g->forces[side], GRASP_CELLS, &stamp,
										 g->bodies[side + 1].position, 1,
										 &g->tactile[side]) != PME_OK)
			return 0;
	}
	return 1;
}
