#include "../DEBUG/YMPME_Log.h"
#include "YMPME_Dynamics.h"
#include "YMPME_Manifold.h"
#include "YMPME_ContactCache.h"
#include "YMPME_CCD.h"
#include <string.h>
#define V YMPME_Vec3
#define add YMPME_VecAdd
#define sub YMPME_VecSub
#define mul YMPME_VecScale
#define dot YMPME_VecDot
#define cross YMPME_VecCross
static float length(pme_vec3 v)
{
	return sqrtf(dot(v, v));
}
static float component(pme_vec3 v, int i)
{
	return i == 0 ? v.x : i == 1 ? v.y : v.z;
}
static pme_vec3 axis(int i)
{
	return V(i == 0, i == 1, i == 2);
}
pme_collider YMPME_Dynamics_Collider(size_t body, pme_collider_type type)
{
	pme_collider c = {0};
	c.body = body;
	c.type = type;
	c.halfSize = V(.5f, .5f, .5f);
	c.radius = .5f;
	c.localOrientation = YMPME_Quaternion_Identity();
	c.friction = .6f;
	c.category = 1;
	c.mask = UINT32_MAX;
	return c;
}
pme_dynamics YMPME_Dynamics_Default(void)
{
	pme_dynamics w = {0};
	w.gravity = V(0, -9.81f, 0);
	w.slop = .0001f;
	w.stabilization = .15f;
	w.maxCorrectionSpeed = .5f;
	w.bounceThreshold = .5f;
	w.iterations = 48;
	return w;
}
static int enabled(const pme_dynamics *w, const pme_collider *a, const pme_collider *b)
{
	if (a->body == b->body || (!w->bodies[a->body].inverseMass && !w->bodies[b->body].inverseMass))
		return 0;
	if (a->group && a->group == b->group)
		return a->group > 0;
	return (a->category & b->mask) && (b->category & a->mask);
}
/* Small geometry-only pose: never copy a full rigid body or move its COM. */
static int zeroQuaternion(pme_quaternion q)
{
	return q.x == 0 && q.y == 0 && q.z == 0 && q.w == 0;
}
static pme_collision_pose shapePose(const pme_collider *c, const pme_rigid_body *b)
{
	pme_collision_pose p = {b->position, b->orientation};
	if (c->localPosition.x != 0 || c->localPosition.y != 0 || c->localPosition.z != 0)
		p.position = add(p.position, YMPME_Rotation_Apply(YMPME_Quaternion_Rotation(b->orientation),
														  c->localPosition));
	pme_quaternion q = c->localOrientation;
	if (!zeroQuaternion(q) && !(q.x == 0 && q.y == 0 && q.z == 0 && q.w == 1))
	{
		p.orientation = YMPME_Quaternion_Multiply(b->orientation, q);
		q = p.orientation;
		float inv = 1 / sqrtf(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
		p.orientation = (pme_quaternion){q.x * inv, q.y * inv, q.z * inv, q.w * inv};
	}
	return p;
}
static int valid(const pme_collider *c, const pme_dynamics *w)
{
	if (c->body >= w->bodyCount || c->type < PME_COLLIDER_SPHERE || c->type > PME_COLLIDER_MESH ||
		!isfinite(c->friction) || c->friction < 0 || !isfinite(c->restitution) ||
		c->restitution < 0 || c->restitution > 1 || !YMPME_VecFinite(c->localPosition) ||
		(!zeroQuaternion(c->localOrientation) && !YMPME_Quaternion_IsValid(c->localOrientation)))
		return 0;
	if (!YMPME_VecFinite(shapePose(c, &w->bodies[c->body]).position))
		return 0;
	if (c->type == PME_COLLIDER_SPHERE)
		return isfinite(c->radius) && c->radius > 0;
	if (c->type == PME_COLLIDER_BOX)
		return YMPME_VecFinite(c->halfSize) && c->halfSize.x > 0 && c->halfSize.y > 0 &&
			   c->halfSize.z > 0;
	if (c->type == PME_COLLIDER_MESH)
	{
		const pme_convex_mesh *m = c->mesh;
		return m && YMPME_ConvexMesh_HasVertices(m) && YMPME_ConvexMesh_HasIndices(m) && m->faceAxes && m->edgeAxes &&
			   m->vertexCount >= 4 && m->vertexCount-1 <= YMPME_INDEX_MAX && m->triangleCount >= 4 &&
			   m->triangleCount <= SIZE_MAX / 3 && m->faceCount > 0 && m->edgeCount > 0 &&
			   YMPME_VecFinite(m->bounds.min) && YMPME_VecFinite(m->bounds.max) &&
			   m->bounds.min.x <= m->bounds.max.x && m->bounds.min.y <= m->bounds.max.y &&
			   m->bounds.min.z <= m->bounds.max.z;
	}
	return w->bodies[c->body].inverseMass == 0;
}
static pme_vec3 extent(const pme_collider *c, const pme_collision_pose *b)
{
	if (c->type == PME_COLLIDER_SPHERE)
		return V(c->radius, c->radius, c->radius);
	pme_rotation r = YMPME_Quaternion_Rotation(b->orientation);
	pme_vec3 e = V(0, 0, 0);
	pme_vec3 half = c->type == PME_COLLIDER_MESH
						? mul(sub(c->mesh->bounds.max, c->mesh->bounds.min), .5f)
						: c->halfSize;
	for (int i = 0; i < 3; ++i)
		e = add(e, mul(V(fabsf(r.axis[i].x), fabsf(r.axis[i].y), fabsf(r.axis[i].z)),
					   component(half, i)));
	return e;
}
static pme_vec3 boundsCenter(const pme_collider *c, const pme_collision_pose *b)
{
	if (c->type != PME_COLLIDER_MESH)
		return b->position;
	return add(b->position,
			   YMPME_Rotation_Apply(YMPME_Quaternion_Rotation(b->orientation),
									mul(add(c->mesh->bounds.min, c->mesh->bounds.max), .5f)));
}
static int broad(const pme_collider *a, const pme_collision_pose *ba, const pme_collider *b,
				 const pme_collision_pose *bb)
{
	if (a->type == PME_COLLIDER_PLANE || b->type == PME_COLLIDER_PLANE)
		return 1;
	pme_vec3 d = sub(boundsCenter(a, ba), boundsCenter(b, bb)),
			 e = add(extent(a, ba), extent(b, bb));
	return fabsf(d.x) <= e.x && fabsf(d.y) <= e.y && fabsf(d.z) <= e.z;
}
static int emit(pme_dynamics *w, size_t ia, size_t ib, pme_vec3 n, pme_vec3 pa, pme_vec3 pb,
				float depth)
{
	if (depth < 0)
		return 1;
	if (w->contactCount == w->contactCapacity)
		return 0;
	pme_dynamic_contact *c = &w->contacts[w->contactCount++];
	memset(c, 0, sizeof(*c));
	c->colliderA = ia;
	c->colliderB = ib;
	c->bodyA = w->colliders[ia].body;
	c->bodyB = w->colliders[ib].body;
	c->point = mul(add(pa, pb), .5f);
	c->normal = n;
	c->penetration = depth;
	c->persistence = 1;
	if (w->cache)
	{
		c->localPointA =
			YMPME_Rotation_InverseApply(YMPME_Quaternion_Rotation(w->bodies[c->bodyA].orientation),
										sub(pa, w->bodies[c->bodyA].position));
		c->localPointB =
			YMPME_Rotation_InverseApply(YMPME_Quaternion_Rotation(w->bodies[c->bodyB].orientation),
										sub(pb, w->bodies[c->bodyB].position));
	}
	c->originA = w->bodies[c->bodyA].position;
	c->originB = w->bodies[c->bodyB].position;
	c->friction = sqrtf(w->colliders[ia].friction) * sqrtf(w->colliders[ib].friction);
	return 1;
}
static int detect(pme_dynamics *w, size_t ia, size_t ib)
{
	const pme_collider *a = &w->colliders[ia], *b = &w->colliders[ib];
	pme_collision_pose poseA = shapePose(a, &w->bodies[a->body]),
					   poseB = shapePose(b, &w->bodies[b->body]);
	const pme_collision_pose *ba = &poseA, *bb = &poseB;
	if (!broad(a, ba, b, bb))
		return 1;
	if (a->type == PME_COLLIDER_MESH || b->type == PME_COLLIDER_MESH ||
		(a->type == PME_COLLIDER_BOX && b->type == PME_COLLIDER_BOX))
	{
		if (b->type == PME_COLLIDER_PLANE || b->type == PME_COLLIDER_SPHERE)
			return detect(w, ib, ia);
		pme_manifold_point points[4];
		size_t count = PME_ConvexManifold(a, ba, b, bb, points);
		for (size_t i = 0; i < count; ++i)
			if (!emit(w, ia, ib, points[i].normal, points[i].pointA, points[i].pointB,
					  points[i].depth))
				return 0;
		return 1;
	}
	/* Canonical order: plane first; otherwise box before sphere. */
	if (b->type == PME_COLLIDER_PLANE ||
		(a->type == PME_COLLIDER_SPHERE && b->type == PME_COLLIDER_BOX))
		return detect(w, ib, ia);
	if (a->type == PME_COLLIDER_PLANE)
	{
		pme_vec3 n = YMPME_Quaternion_Rotation(ba->orientation).axis[1];
		if (b->type == PME_COLLIDER_SPHERE)
		{
			float d = dot(sub(bb->position, ba->position), n);
			return emit(w, ia, ib, n, sub(bb->position, mul(n, d)),
						sub(bb->position, mul(n, b->radius)), b->radius - d);
		}
		pme_rotation r = YMPME_Quaternion_Rotation(bb->orientation);
		for (int i = 0; i < 8; ++i)
		{
			pme_vec3 p =
				add(bb->position, YMPME_Rotation_Apply(r, V((i & 1 ? 1 : -1) * b->halfSize.x,
															(i & 2 ? 1 : -1) * b->halfSize.y,
															(i & 4 ? 1 : -1) * b->halfSize.z)));
			float d = dot(sub(p, ba->position), n);
			if (!emit(w, ia, ib, n, sub(p, mul(n, d)), p, -d))
				return 0;
		}
		return 1;
	}
	if (a->type == PME_COLLIDER_SPHERE)
	{
		pme_vec3 d = sub(bb->position, ba->position);
		float len = length(d);
		pme_vec3 n = len > 1e-8f ? mul(d, 1 / len) : V(1, 0, 0);
		return emit(w, ia, ib, n, add(ba->position, mul(n, a->radius)),
					sub(bb->position, mul(n, b->radius)), a->radius + b->radius - len);
	}
	pme_rotation r = YMPME_Quaternion_Rotation(ba->orientation);
	pme_vec3 p = YMPME_Rotation_InverseApply(r, sub(bb->position, ba->position));
	pme_vec3 q = V(YMPME_Clamp(p.x, -a->halfSize.x, a->halfSize.x),
				   YMPME_Clamp(p.y, -a->halfSize.y, a->halfSize.y),
				   YMPME_Clamp(p.z, -a->halfSize.z, a->halfSize.z));
	pme_vec3 d = sub(p, q), n;
	float len = length(d), depth;
	if (len > 1e-8f)
	{
		n = mul(d, 1 / len);
		depth = b->radius - len;
	}
	else
	{
		int k = 0;
		float nearest = a->halfSize.x - fabsf(p.x);
		for (int i = 1; i < 3; ++i)
			if (component(a->halfSize, i) - fabsf(component(p, i)) < nearest)
			{
				k = i;
				nearest = component(a->halfSize, i) - fabsf(component(p, i));
			}
		n = mul(axis(k), component(p, k) < 0 ? -1 : 1);
		q = add(p, mul(n, nearest));
		depth = b->radius + nearest;
	}
	n = YMPME_Rotation_Apply(r, n);
	q = add(ba->position, YMPME_Rotation_Apply(r, q));
	return emit(w, ia, ib, n, q, sub(bb->position, mul(n, b->radius)), depth);
}
static void row(pme_joint_row *r, const pme_dynamic_contact *c, const pme_rigid_body *a,
				const pme_rigid_body *b, pme_vec3 direction)
{
	memset(r, 0, sizeof(*r));
	r->linearA = mul(direction, -1);
	r->linearB = direction;
	r->angularA = mul(cross(sub(c->point, a->position), direction), -1);
	r->angularB = cross(sub(c->point, b->position), direction);
	r->responseLinearA = mul(r->linearA, a->inverseMass);
	r->responseLinearB = mul(r->linearB, b->inverseMass);
	r->responseAngularA = YMPME_Rigid_InverseInertia(a, r->angularA);
	r->responseAngularB = YMPME_Rigid_InverseInertia(b, r->angularB);
	float k = dot(r->linearA, r->responseLinearA) + dot(r->linearB, r->responseLinearB) +
			  dot(r->angularA, r->responseAngularA) + dot(r->angularB, r->responseAngularB);
	r->inverseEffectiveMass = k > 1e-12f ? 1 / k : 0;
}
static float speed(const pme_joint_row *r, const pme_rigid_body *a, const pme_rigid_body *b)
{
	return dot(r->linearA, a->velocity) + dot(r->angularA, a->angularVelocity) +
		   dot(r->linearB, b->velocity) + dot(r->angularB, b->angularVelocity);
}
static void apply(pme_joint_row *r, pme_rigid_body *a, pme_rigid_body *b, float impulse)
{
	float delta = impulse - r->impulse;
	r->impulse = impulse;
	a->velocity = add(a->velocity, mul(r->responseLinearA, delta));
	a->angularVelocity = add(a->angularVelocity, mul(r->responseAngularA, delta));
	b->velocity = add(b->velocity, mul(r->responseLinearB, delta));
	b->angularVelocity = add(b->angularVelocity, mul(r->responseAngularB, delta));
}
static void prepare(pme_dynamics *w, pme_dynamic_contact *c, float dt)
{
	pme_rigid_body *a = &w->bodies[c->bodyA], *b = &w->bodies[c->bodyB];
	pme_vec3 t = cross(c->normal, fabsf(c->normal.x) < .7f ? V(1, 0, 0) : V(0, 1, 0));
	t = mul(t, 1 / length(t));
	row(&c->rows[0], c, a, b, c->normal);
	row(&c->rows[1], c, a, b, t);
	row(&c->rows[2], c, a, b, cross(c->normal, t));
	float vn = speed(&c->rows[0], a, b);
	float bounce = vn < -w->bounceThreshold ? -vn * fminf(w->colliders[c->colliderA].restitution,
														  w->colliders[c->colliderB].restitution)
											: 0;
	c->rows[0].target =
		fmaxf(bounce, fminf(w->maxCorrectionSpeed,
							w->stabilization * fmaxf(0, c->penetration - w->slop) / dt));
	const pme_joint_row *u = &c->rows[1], *v = &c->rows[2];
	c->tangentK01 = dot(u->linearA, v->responseLinearA) + dot(u->linearB, v->responseLinearB) +
					dot(u->angularA, v->responseAngularA) + dot(u->angularB, v->responseAngularB);
}
static void solve(pme_dynamics *w, pme_dynamic_contact *c)
{
	pme_rigid_body *a = &w->bodies[c->bodyA], *b = &w->bodies[c->bodyB];
	pme_joint_row *n = &c->rows[0], *u = &c->rows[1], *v = &c->rows[2];
	apply(n, a, b, fmaxf(0, n->impulse + (n->target - speed(n, a, b)) * n->inverseEffectiveMass));
	float su = speed(u, a, b), sv = speed(v, a, b), ku = u->inverseEffectiveMass,
		  kv = v->inverseEffectiveMass, k = c->tangentK01;
	float den = 1 - k * k * ku * kv;
	float du = den > 1e-6f ? (-su * ku + k * ku * kv * sv) / den : -su * ku;
	float dv = den > 1e-6f ? (-sv * kv + k * ku * kv * su) / den : -sv * kv;
	float x = u->impulse + du, y = v->impulse + dv, mag = hypotf(x, y),
		  limit = c->friction * n->impulse;
	if (mag > limit && mag > 0)
	{
		x *= limit / mag;
		y *= limit / mag;
	}
	apply(u, a, b, x);
	apply(v, a, b, y);
}
pme_result YMPME_Dynamics_Step(pme_dynamics *w, float dt, double time)
{
	if (!w || (w->colliderCount && !w->colliders) || (w->contactCapacity && !w->contacts) ||
		(w->cache && !PME_ContactCache_Valid(w->cache)) || !isfinite(w->slop) || w->slop < 0 ||
		!isfinite(w->stabilization) || w->stabilization < 0 || w->stabilization > 1 ||
		!isfinite(w->maxCorrectionSpeed) || w->maxCorrectionSpeed < 0 ||
		!isfinite(w->bounceThreshold) || w->bounceThreshold < 0 || !isfinite(w->time) ||
		time < w->time + dt * .5 ||
		YMPME_Joint_ValidateStep(w->bodies, w->bodyCount, w->joints, w->jointCount, w->gravity, dt,
								 w->iterations, time) != PME_OK)
		return YMPME_REPORT(PME_INVALID_ARGUMENT, "Invalid dynamics world, buffers, solver parameters, bodies, joints or time");
	if (w->jointCache)
	{
		pme_result result = YMPME_Joint_ValidateCache(w->jointCache, w->jointCount);
		if (result != PME_OK) return YMPME_REPORT(result, "Invalid joint cache or insufficient capacity");
	}
	for (size_t i = 0; i < w->colliderCount; ++i)
		if (!valid(&w->colliders[i], w))
			return YMPME_REPORT(PME_INVALID_ARGUMENT, "Invalid collider shape, mesh, body index or transform");
	if (w->ccd)
	{
		pme_result result = PME_CCD_Validate(w);
		if (result != PME_OK) return YMPME_REPORT(result, "Invalid CCD configuration or insufficient capacity");
	}
	w->contactCount = 0;
	for (size_t i = 0; i < w->colliderCount; ++i)
		for (size_t j = i + 1; j < w->colliderCount; ++j)
			if (enabled(w, &w->colliders[i], &w->colliders[j]) && !detect(w, i, j))
			{
				w->contactCount = 0;
				return YMPME_REPORT(PME_CAPACITY_EXCEEDED, "Contact buffer capacity exceeded");
			}
	if (w->cache && w->cache->capacity < w->contactCount)
	{
		w->contactCount = 0;
		return YMPME_REPORT(PME_CAPACITY_EXCEEDED, "Contact cache capacity exceeded");
	}
	for (size_t i = 0; i < w->contactCount; ++i)
	{
		const pme_dynamic_contact *c = &w->contacts[i];
		if (!YMPME_VecFinite(c->point) || !YMPME_VecFinite(c->normal) ||
			!isfinite(c->penetration) || !isfinite(c->friction) ||
			(w->cache && (!YMPME_VecFinite(c->localPointA) || !YMPME_VecFinite(c->localPointB))))
		{
			w->contactCount = 0;
			return YMPME_REPORT(PME_INVALID_ARGUMENT, "Contact geometry or material contains non-finite values");
		}
	}
	if (w->ccd) { w->ccd->count = 0; w->ccd->clampedBodies = 0; }
	for (size_t i = 0; i < w->bodyCount; ++i)
		YMPME_Rigid_Begin(&w->bodies[i], w->gravity, dt);
	YMPME_Joint_Prepare(w->bodies, w->joints, w->jointCount, dt);
	for (size_t i = 0; i < w->contactCount; ++i)
		prepare(w, &w->contacts[i], dt);
	if (w->jointCache)
	{
		YMPME_Joint_WarmStart(w->bodies, w->bodyCount, w->joints, w->jointCount,
							 w->jointCache, dt, w->time);
	}
	if (w->cache)
		PME_ContactCache_WarmStart(w, dt);
	for (unsigned k = 0; k < w->iterations; ++k)
	{
		YMPME_Joint_SolveIteration(w->bodies, w->joints, w->jointCount);
		for (size_t i = 0; i < w->contactCount; ++i)
			solve(w, &w->contacts[i]);
	}
	for (size_t i = 0; i < w->contactCount; ++i)
	{
		pme_dynamic_contact *c = &w->contacts[i];
		c->normalImpulse = c->rows[0].impulse;
		c->tangentImpulse = add(mul(c->rows[1].linearB, c->rows[1].impulse),
								mul(c->rows[2].linearB, c->rows[2].impulse));
		c->reactionB.force = mul(add(mul(c->normal, c->normalImpulse), c->tangentImpulse), 1 / dt);
		c->reactionA.force = mul(c->reactionB.force, -1);
		c->reactionA.torque = cross(sub(c->point, c->originA), c->reactionA.force);
		c->reactionB.torque = cross(sub(c->point, c->originB), c->reactionB.force);
	}
	if (w->jointCache)
	{
		YMPME_Joint_CacheStore(w->bodies, w->bodyCount, w->joints, w->jointCount,
							  w->jointCache, dt, time);
	}
	if (w->cache)
		PME_ContactCache_Store(w, dt, time);
	for (size_t i = 0; i < w->bodyCount; ++i)
	{
		if (w->ccd && w->ccd->enabled[i]) PME_CCD_Integrate(w, i, dt, time);
		else YMPME_Rigid_End(&w->bodies[i], dt, time);
	}
	YMPME_Joint_Finish(w->bodies, w->joints, w->jointCount, dt, time);
	w->time = time;
	w->interval = dt;
	return PME_OK;
}
