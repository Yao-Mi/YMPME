#include "soft_finger.h"
#define V YMPME_Vec3
#define add YMPME_VecAdd
#define sub YMPME_VecSub
#define mul YMPME_VecScale
#define dot YMPME_VecDot
static float forceMode(const soft_finger *s, pme_vec3 p, pme_vec3 force)
{
	const pme_rigid_body *b = &s->grasp.bodies[GRASP_OBJECT];
	pme_rotation r = YMPME_Quaternion_Rotation(b->orientation);
	pme_vec3 local = YMPME_Rotation_InverseApply(r, sub(p, b->position));
	return dot(YMPME_Rotation_InverseApply(r, force), YMPME_SoftBall_Mode(&s->ball, local));
}
int SoftFinger_BallStep(soft_finger *s)
{
	grasp_lab *g = &s->grasp;
	pme_rigid_body *b = &g->bodies[GRASP_OBJECT];
	float force = 0;
	/* Explicit staggered coupling: previous substep contact loads drive this mode. */
	for (int side = 0; side < 2; ++side)
		for (int k = 0; k < GRASP_CELLS; ++k)
			force += forceMode(s, s->nodes[side][k].position, mul(s->nodes[side][k].force, -1));
	for (size_t k = 0; k < g->world.contactCount; ++k)
	{
		const pme_dynamic_contact *c = &g->contacts[k];
		if (c->bodyA == GRASP_OBJECT)
			force += forceMode(s, c->point, c->reactionA.force);
		if (c->bodyB == GRASP_OBJECT)
			force += forceMode(s, c->point, c->reactionB.force);
	}
	s->ballForce = force;
	if (YMPME_SoftBall_Step(&s->ball, force, GRASP_DT) != PME_OK)
		return 0;
	pme_vec3 scale = YMPME_SoftBall_Scale(&s->ball), axes = mul(scale, s->ball.radius);
	/* Rebuild an inactive convex proxy, then swap it into the collider between
	 * steps. The borrowed mesh used by the dynamics solver is never mutated. */
	s->ballProxy = 1 - s->ballProxy;
	soft_ball_proxy *p = &s->ballGeometry[s->ballProxy];
	p->vertices[0] = V(0, axes.y, 0);
	p->vertices[25] = V(0, -axes.y, 0);
	for (int row = 0; row < 3; ++row)
		for (int col = 0; col < 8; ++col)
		{
			float a = (row + 1) * 3.14159265359f / 4, t = col * 3.14159265359f / 4;
			p->vertices[1 + row * 8 + col] =
				V(axes.x * sinf(a) * cosf(t), axes.y * cosf(a), axes.z * sinf(a) * sinf(t));
		}
	size_t count = 0;
	for (int col = 0; col < 8; ++col)
	{
		int next = (col + 1) % 8;
		p->indices[count++] = 0;
		p->indices[count++] = 1 + col;
		p->indices[count++] = 1 + next;
		p->indices[count++] = 25;
		p->indices[count++] = 17 + next;
		p->indices[count++] = 17 + col;
		for (int row = 0; row < 2; ++row)
		{
			int a = 1 + row * 8 + col, bb = 1 + row * 8 + next, c = a + 8, d = bb + 8;
			p->indices[count++] = a;
			p->indices[count++] = c;
			p->indices[count++] = bb;
			p->indices[count++] = bb;
			p->indices[count++] = c;
			p->indices[count++] = d;
		}
	}
	if (YMPME_ConvexMesh_Init(&p->mesh, p->vertices, 26, p->indices, 48, p->faces, 48, p->edges,
							  144) != PME_OK)
		return 0;
	g->colliders[2].type = PME_COLLIDER_MESH;
	g->colliders[2].mesh = &p->mesh;
	if (g->world.cache)
		YMPME_Dynamics_CacheClear(g->world.cache);
	/* Uniform affine ellipsoid inertia; preserve angular momentum across shape update. */
	pme_rotation r = YMPME_Quaternion_Rotation(b->orientation);
	pme_vec3 omega = YMPME_Rotation_InverseApply(r, b->angularVelocity);
	pme_vec3 momentum = V(b->inertia.x * omega.x, b->inertia.y * omega.y, b->inertia.z * omega.z);
	b->inertia = V(b->mass * (axes.y * axes.y + axes.z * axes.z) / 5,
				   b->mass * (axes.x * axes.x + axes.z * axes.z) / 5,
				   b->mass * (axes.x * axes.x + axes.y * axes.y) / 5);
	b->inverseInertia = V(1 / b->inertia.x, 1 / b->inertia.y, 1 / b->inertia.z);
	b->angularVelocity = YMPME_Rotation_Apply(r, V(momentum.x * b->inverseInertia.x,
												   momentum.y * b->inverseInertia.y,
												   momentum.z * b->inverseInertia.z));
	return 1;
}
