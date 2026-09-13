#include "YMPME.h"
#include "CORE/YMPME_MeshSAT.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define V YMPME_Vec3
#define CHECK(x)                                                                                   \
	do                                                                                             \
	{                                                                                              \
		if (!(x))                                                                                  \
		{                                                                                          \
			fprintf(stderr, "line %d: %s\n", __LINE__, #x);                                        \
			exit(1);                                                                               \
		}                                                                                          \
	} while (0)
typedef struct fixture_
{
	pme_rigid_body b[2];
	pme_collider c[2];
	pme_dynamic_contact contacts[32];
	pme_dynamics w;
	pme_convex_mesh mesh;
	pme_vec3 vertices[8], faces[12], edges[36];
} fixture;
static const uint32_t ix[36] = {0, 4, 6, 0, 6, 2, 1, 3, 7, 1, 7, 5, 0, 1, 5, 0, 5, 4,
								2, 6, 7, 2, 7, 3, 0, 2, 3, 0, 3, 1, 4, 5, 7, 4, 7, 6};
static float norm(pme_vec3 p) { return sqrtf(YMPME_VecDot(p, p)); }
static void init(fixture *f, int meshA, int meshB)
{
	memset(f, 0, sizeof(*f));
	for (int i = 0; i < 8; ++i)
		f->vertices[i] = V((i & 1 ? .5f : -.5f), (i & 2 ? .5f : -.5f), (i & 4 ? .5f : -.5f));
	CHECK(YMPME_ConvexMesh_Init(&f->mesh, f->vertices, 8, ix, 12, f->faces, 12, f->edges, 36) ==
		  PME_OK);
	for (int i = 0; i < 2; ++i)
	{
		CHECK(YMPME_Rigid_Init(&f->b[i], i ? 1 : 0, V(.1f, .1f, .1f), V(0, i ? .5f : -.5f, 0),
							   YMPME_Quaternion_Identity()) == PME_OK);
		f->c[i] =
			YMPME_Dynamics_Collider(i, (i ? meshB : meshA) ? PME_COLLIDER_MESH : PME_COLLIDER_BOX);
		f->c[i].mesh = &f->mesh;
	}
	f->w = YMPME_Dynamics_Default();
	f->w.bodies = f->b;
	f->w.bodyCount = 2;
	f->w.colliders = f->c;
	f->w.colliderCount = 2;
	f->w.contacts = f->contacts;
	f->w.contactCapacity = 32;
}
static void step(fixture *f)
{
	CHECK(YMPME_Dynamics_Step(&f->w, 1.f / 240, f->w.time + 1. / 240) == PME_OK);
}
static void rest(void)
{
	for (int a = 0; a < 2; ++a)
		for (int b = 0; b < 2; ++b)
		{
			fixture f;
			init(&f, a, b);
			for (int i = 0; i < 720; ++i)
				step(&f);
			CHECK(f.w.contactCount >= 3 && f.w.contactCount <= 4);
			CHECK(fabsf(f.b[1].position.y - .5f) < .002f && norm(f.b[1].angularVelocity) < .005f);
			pme_vec3 force = V(0, 0, 0);
			for (size_t i = 0; i < f.w.contactCount; ++i)
			{
				const pme_dynamic_contact *c = &f.contacts[i];
				force = YMPME_VecAdd(force, c->reactionB.force);
				CHECK(norm(YMPME_VecAdd(c->reactionA.force, c->reactionB.force)) < 1e-6f);
				CHECK(fabsf(c->point.x) <= .5001f && fabsf(c->point.z) <= .5001f);
				for (size_t j = 0; j < i; ++j)
					CHECK(norm(YMPME_VecSub(c->point, f.contacts[j].point)) > 1e-5f);
			}
			CHECK(fabsf(force.y - 9.81f) < .03f);
			printf("rest box/mesh %d%d: %zu contacts, y %.6f Fy %.4f\n", a, b, f.w.contactCount,
				   f.b[1].position.y, force.y);
		}
}
static void rotated_and_capacity(void)
{
	fixture f;
	init(&f, 0, 0);
	f.w.gravity = V(0, 0, 0);
	f.c[0].halfSize = f.c[1].halfSize = V(1, .05f, .1f);
	f.b[0].position = V(0, 0, 0);
	f.b[1].position = V(-.15f, .15f, 0);
	CHECK(YMPME_Quaternion_AxisAngle(V(0, 0, 1), .7853981634f, &f.b[0].orientation) == PME_OK);
	f.b[1].orientation = f.b[0].orientation;
	step(&f);
	CHECK(f.w.contactCount == 0); /* AABBs overlap, thin rotated boxes do not. */
	f.b[1].position = V(-.04f, .04f, 0);
	step(&f);
	CHECK(f.w.contactCount >= 2);
	pme_rigid_body old[2];
	memcpy(old, f.b, sizeof old);
	double time = f.w.time;
	f.w.contactCapacity = 1;
	CHECK(YMPME_Dynamics_Step(&f.w, 1.f / 240, time + 1. / 240) == PME_CAPACITY_EXCEEDED);
	CHECK(!memcmp(old, f.b, sizeof old) && f.w.time == time);
	f.w.contactCapacity = 32;
	f.c[0].group = f.c[1].group = -5;
	step(&f);
	CHECK(f.w.contactCount == 0);
	/* Null Mesh is rejected before changing any bodies. */
	f.c[0].type = PME_COLLIDER_MESH;
	f.c[0].mesh = NULL;
	memcpy(old, f.b, sizeof old);
	CHECK(YMPME_Dynamics_Step(&f.w, 1.f / 240, f.w.time + 1. / 240) == PME_INVALID_ARGUMENT);
	CHECK(!memcmp(old, f.b, sizeof old));
}
static void sphere_and_plane(void)
{
	fixture f;
	init(&f, 1, 0);
	f.w.gravity = V(0, 0, 0);
	f.b[0].position = V(0, 0, 0);
	f.c[1].type = PME_COLLIDER_SPHERE;
	f.b[1].position = V(1, 1, 0);
	step(&f);
	CHECK(f.w.contactCount == 0);
	f.b[1].position = V(.9f, 0, 0);
	step(&f);
	CHECK(f.w.contactCount == 1);
	CHECK(fabsf(f.contacts[0].penetration - .1f) < 1e-5f);
	CHECK(f.b[1].velocity.x > 0 && norm(f.b[1].angularVelocity) < 1e-5f);
	f.b[1].position = V(0, 0, 0);
	f.c[1].radius = .1f;
	step(&f);
	CHECK(f.w.contactCount == 1);
	CHECK(fabsf(f.contacts[0].penetration - .6f) < 1e-5f);
	init(&f, 0, 1);
	f.c[0].type = PME_COLLIDER_PLANE;
	f.b[0].position = V(0, 0, 0);
	for (int i = 0; i < 480; ++i)
		step(&f);
	CHECK(f.w.contactCount == 4 && fabsf(f.b[1].position.y - .5f) < .002f);
	/* A translated local mesh bound must not be centred at the body's origin. */
	init(&f, 1, 0);
	for (int i = 0; i < 8; ++i)
		f.vertices[i].x += 2;
	CHECK(YMPME_ConvexMesh_Init(&f.mesh, f.vertices, 8, ix, 12, f.faces, 12, f.edges, 36) ==
		  PME_OK);
	f.w.gravity = V(0, 0, 0);
	f.b[0].position = V(-2, 0, 0);
	f.b[1].position = V(.9f, 0, 0);
	f.c[1].type = PME_COLLIDER_SPHERE;
	step(&f);
	CHECK(f.w.contactCount == 1);
}
static uint32_t seed = 1234567;
static float randomFloat(void)
{
	seed = seed * 1664525u + 1013904223u;
	return (seed >> 8) * (1.f / 16777216);
}
static void rotated_features(void)
{
	unsigned hits = 0, single = 0;
	for (int trial = 0; trial < 600; ++trial)
	{
		fixture f;
		init(&f, trial & 1, (trial >> 1) & 1);
		f.w.gravity = V(0, 0, 0);
		f.w.stabilization = 0;
		f.b[0].position = V(0, 0, 0);
		f.b[1].position = V((randomFloat() - .5f) * 2.6f, (randomFloat() - .5f) * 2.6f,
							(randomFloat() - .5f) * 2.6f);
		for (int i = 0; i < 2; ++i)
			CHECK(YMPME_Quaternion_AxisAngle(
					  V(.2f + randomFloat(), .2f + randomFloat(), .2f + randomFloat()),
					  randomFloat() * 6.28f, &f.b[i].orientation) == PME_OK);
		pme_body proxy[2] = {{0}};
		for (int i = 0; i < 2; ++i)
		{
			proxy[i].state.shape = PME_CONVEX_MESH;
			proxy[i].state.mesh = &f.mesh;
			proxy[i].state.position = f.b[i].position;
			proxy[i].state.rotation = YMPME_Quaternion_Rotation(f.b[i].orientation);
		}
		pme_contact expected;
		int collision = YMPME_MeshSAT(&proxy[0], &proxy[1], &expected);
		step(&f);
		CHECK((f.w.contactCount > 0) == collision);
		CHECK(f.w.contactCount <= 4);
		if (collision)
		{
			++hits;
			if (f.w.contactCount == 1)
				++single;
		}
		for (size_t i = 0; i < f.w.contactCount; ++i)
		{
			CHECK(YMPME_VecFinite(f.contacts[i].point));
			CHECK(fabsf(norm(f.contacts[i].normal) - 1) < 1e-4f);
			CHECK(f.contacts[i].penetration >= 0 &&
				  f.contacts[i].penetration <= expected.penetration + 1e-3f);
		}
	}
	CHECK(hits > 100 && single > 20);
	printf("random rotated cases: hits %u single-feature %u\n", hits, single);
}
int main(void)
{
	rest();
	rotated_features();
	rotated_and_capacity();
	sphere_and_plane();
	puts("CONVEX DYNAMICS PASS");
	return 0;
}
