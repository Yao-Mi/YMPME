#include "YMPME.h"
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
static float norm(pme_vec3 v)
{
	return sqrtf(YMPME_VecDot(v, v));
}
static pme_quaternion rot(float a)
{
	pme_quaternion q;
	CHECK(YMPME_Quaternion_AxisAngle(V(0, 0, 1), a, &q) == PME_OK);
	return q;
}
typedef struct fixture_
{
	pme_rigid_body b[5];
	pme_collider c[5];
	pme_dynamic_contact contacts[64];
	pme_dynamics w;
	pme_contact_cache cache;
	pme_contact_cache_entry entries[64];
} fixture;
static void init(fixture *f, int count, int warm)
{
	memset(f, 0, sizeof(*f));
	for (int i = 0; i < count; ++i)
	{
		CHECK(YMPME_Rigid_Init(&f->b[i], i ? 1 : 0, V(.16f, .16f, .16f), V(0, i - .5f, 0),
							   rot(0)) == PME_OK);
		f->c[i] = YMPME_Dynamics_Collider(i, PME_COLLIDER_BOX);
	}
	f->w = YMPME_Dynamics_Default();
	f->w.bodies = f->b;
	f->w.bodyCount = count;
	f->w.colliders = f->c;
	f->w.colliderCount = count;
	f->w.contacts = f->contacts;
	f->w.contactCapacity = 64;
	CHECK(YMPME_Dynamics_CacheInit(&f->cache, f->entries, 64) == PME_OK);
	if (warm)
		f->w.cache = &f->cache;
}
static void step(fixture *f, float dt)
{
	CHECK(YMPME_Dynamics_Step(&f->w, dt, f->w.time + (double)dt) == PME_OK);
}
static void compound(void)
{
	fixture f;
	init(&f, 2, 1);
	f.c[0].type = PME_COLLIDER_PLANE;
	f.b[0].position = V(0, 0, 0);
	f.c[1].type = PME_COLLIDER_SPHERE;
	f.c[1].localPosition = V(.6f, 0, 0);
	f.c[2] = f.c[1];
	f.c[2].localPosition.x = -.6f;
	f.w.colliderCount = 3;
	for (int i = 0; i < 480; ++i)
		step(&f, 1.f / 240);
	CHECK(f.w.contactCount == 2);
	CHECK(f.cache.hits == 2);
	CHECK(fabsf(f.b[1].position.y - .5f) < .001f);
	CHECK(norm(f.b[1].angularVelocity) < .001f);
	pme_vec3 force = V(0, 0, 0), torque = V(0, 0, 0);
	for (size_t i = 0; i < f.w.contactCount; ++i)
	{
		force = YMPME_VecAdd(force, f.contacts[i].reactionB.force);
		torque = YMPME_VecAdd(torque, f.contacts[i].reactionB.torque);
	}
	CHECK(fabsf(force.y - 9.81f) < .01f);
	CHECK(norm(torque) < .001f);
	/* A single offset sphere produces a torque about the true COM. */
	init(&f, 2, 0);
	f.c[0].type = PME_COLLIDER_PLANE;
	f.b[0].position = V(0, 0, 0);
	f.b[1].position = V(0, .49f, 0);
	f.c[1].type = PME_COLLIDER_SPHERE;
	f.c[1].localPosition = V(.6f, 0, 0);
	f.c[0].friction = f.c[1].friction = 0;
	step(&f, 1.f / 240);
	CHECK(f.w.contactCount == 1);
	CHECK(f.b[1].angularVelocity.z > 0);
	CHECK(fabsf(f.contacts[0].reactionB.torque.z - .6f * f.contacts[0].reactionB.force.y) < .001f);
}
static const uint32_t ix[36] = {0, 4, 6, 0, 6, 2, 1, 3, 7, 1, 7, 5, 0, 1, 5, 0, 5, 4,
								2, 6, 7, 2, 7, 3, 0, 2, 3, 0, 3, 1, 4, 5, 7, 4, 7, 6};
static void transformed_geometry(void)
{
	pme_vec3 vertices[8], faces[12], edges[36];
	pme_convex_mesh mesh;
	for (int i = 0; i < 8; ++i)
		vertices[i] = V((i & 1 ? .5f : -.5f), (i & 2 ? .5f : -.5f), (i & 4 ? .5f : -.5f));
	CHECK(YMPME_ConvexMesh_Init(&mesh, vertices, 8, ix, 12, faces, 12, edges, 36) == PME_OK);
	const pme_collider_type types[] = {PME_COLLIDER_SPHERE, PME_COLLIDER_BOX, PME_COLLIDER_MESH,
									   PME_COLLIDER_PLANE};
	for (int a = 0; a < 4; ++a)
		for (int b = 0; b < 3; ++b)
		{
			fixture plain, local;
			init(&plain, 2, 0);
			init(&local, 2, 0);
			for (int i = 0; i < 2; ++i)
			{
				plain.c[i].type = local.c[i].type = types[i ? b : a];
				plain.c[i].mesh = local.c[i].mesh = &mesh;
				pme_vec3 position =
					i ? (a == 3 ? V(.1f, .35f, .03f) : V(.8f, .1f, .03f)) : V(0, 0, 0);
				pme_quaternion desired = rot(i ? .27f : .14f), bodyRotation = rot(i ? -.7f : 1.1f);
				plain.b[i].position = position;
				plain.b[i].orientation = desired;
				local.b[i].position = V(i ? 3 : -2, -2, 1);
				local.b[i].orientation = bodyRotation;
				local.c[i].localPosition =
					YMPME_Rotation_InverseApply(YMPME_Quaternion_Rotation(bodyRotation),
												YMPME_VecSub(position, local.b[i].position));
				local.c[i].localOrientation =
					YMPME_Quaternion_Multiply(YMPME_Quaternion_Conjugate(bodyRotation), desired);
			}
			plain.w.gravity = local.w.gravity = V(0, 0, 0);
			plain.w.stabilization = local.w.stabilization = 0;
			step(&plain, 1.f / 240);
			step(&local, 1.f / 240);
			CHECK(plain.w.contactCount > 0);
			CHECK(plain.w.contactCount == local.w.contactCount);
			for (size_t i = 0; i < plain.w.contactCount; ++i)
			{
				float best = 1e10f;
				for (size_t j = 0; j < local.w.contactCount; ++j)
				{
					float error =
						norm(YMPME_VecSub(plain.contacts[i].point, local.contacts[j].point)) +
						norm(YMPME_VecSub(plain.contacts[i].normal, local.contacts[j].normal)) +
						fabsf(plain.contacts[i].penetration - local.contacts[j].penetration);
					best = fminf(best, error);
				}
				CHECK(best < .0001f);
			}
		}
	fixture f;
	init(&f, 2, 1);
	f.c[1].localOrientation = (pme_quaternion){0};
	step(&f, 1.f / 240); /* legacy zero init */
	pme_rigid_body saved[5];
	memcpy(saved, f.b, sizeof saved);
	double time = f.w.time;
	pme_contact_cache cache = f.cache;
	f.c[1].localPosition.x = NAN;
	CHECK(YMPME_Dynamics_Step(&f.w, 1.f / 240, time + 1. / 240) == PME_INVALID_ARGUMENT);
	CHECK(!memcmp(saved, f.b, sizeof saved));
	CHECK(!memcmp(&cache, &f.cache, sizeof cache));
	f.c[1].localPosition.x = 0;
	f.c[1].localOrientation = (pme_quaternion){0, 0, 0, 2};
	CHECK(YMPME_Dynamics_Step(&f.w, 1.f / 240, time + 1. / 240) == PME_INVALID_ARGUMENT);
}
static void cache_lifecycle(void)
{
	fixture f;
	init(&f, 2, 1);
	f.w.iterations = 8;
	for (int i = 0; i < 120; ++i)
		step(&f, 1.f / 240);
	CHECK(f.cache.count == 4 && f.cache.hits == 4);
	float force = 0;
	for (size_t i = 0; i < 4; ++i)
	{
		CHECK(f.contacts[i].persistence > 100);
		force += f.contacts[i].reactionB.force.y;
	}
	CHECK(fabsf(force - 9.81f) < .01f); /* full solved impulse, including warm seed, exactly once */
	f.b[1].load.force = V(2, 0, 0);
	step(&f, 1.f / 240);
	f.c[0].friction = f.c[1].friction = 0;
	step(&f, 1.f / 240);
	for (size_t i = 0; i < f.w.contactCount; ++i)
		CHECK(norm(f.contacts[i].tangentImpulse) < 1e-6f);
	step(&f, 1.f / 120);
	CHECK(f.cache.hits > 0); /* dt ratio 2 reuses scaled impulses */
	step(&f, 1.f / 480);
	CHECK(f.cache.hits == 0); /* ratio 1/4 discards them */
	YMPME_Dynamics_CacheClear(&f.cache);
	CHECK(!f.cache.count && !f.cache.hits && f.cache.capacity == 64);
	step(&f, 1.f / 240);
	CHECK(f.cache.hits == 0);
	/* Failure before solve must not alter previous cache, rigid bodies or clock. */
	pme_rigid_body saved[5];
	memcpy(saved, f.b, sizeof saved);
	f.cache.capacity = 1;
	pme_contact_cache cache = f.cache;
	pme_contact_cache_entry entries[64];
	memcpy(entries, f.entries, sizeof entries);
	/* count greater than capacity is an invalid descriptor */
	CHECK(YMPME_Dynamics_Step(&f.w, 1.f / 240, f.w.time + 1. / 240) == PME_INVALID_ARGUMENT);
	CHECK(!memcmp(&cache, &f.cache, sizeof cache));
	CHECK(!memcmp(saved, f.b, sizeof saved));
	CHECK(!memcmp(entries, f.entries, sizeof entries));
	YMPME_Dynamics_CacheClear(&f.cache);
	cache = f.cache;
	CHECK(YMPME_Dynamics_Step(&f.w, 1.f / 240, f.w.time + 1. / 240) == PME_CAPACITY_EXCEEDED);
	CHECK(!memcmp(&cache, &f.cache, sizeof cache));
	CHECK(!memcmp(saved, f.b, sizeof saved));
	f.cache.capacity = 64;
	step(&f, 1.f / 240);
	f.c[1].mask = 0;
	step(&f, 1.f / 240);
	CHECK(f.cache.count == 0 && f.cache.hits == 0);
	f.c[1].mask = UINT32_MAX;
	step(&f, 1.f / 240);
	CHECK(f.cache.hits == 0);
	/* A separate world sharing the cache cannot inherit impulses. */
	fixture other;
	init(&other, 2, 0);
	other.w.cache = &f.cache;
	other.w.time = f.w.time;
	step(&other, 1.f / 240);
	CHECK(f.cache.hits == 0);
}
static float stack(int warm)
{
	fixture f;
	init(&f, 5, warm);
	f.w.iterations = 4;
	size_t hits = 0;
	for (int i = 0; i < 720; ++i)
	{
		step(&f, 1.f / 240);
		hits += f.cache.hits;
	}
	float error = 0;
	for (int i = 1; i < 5; ++i)
		error += fabsf(f.b[i].position.y - (i - .5f));
	printf("stack %s: summed height error %.6f m, hits %zu\n", warm ? "warm" : "cold", error, hits);
	if (warm)
		CHECK(hits > 1000);
	return error;
}
static void joint_contact_cache(void)
{
    fixture f;
    pme_joint j, savedJoint;
    pme_joint_cache cache, savedCache;
    pme_joint_cache_entry entry, savedEntry;
    pme_quaternion frame;
    init(&f, 2, 1);
    CHECK(YMPME_Quaternion_AxisAngle(V(1,0,0), 1.5707963268f, &frame) == PME_OK);
    CHECK(YMPME_Joint_Init(&j, PME_JOINT_PRISMATIC, f.b, 2, PME_JOINT_WORLD, 1,
                          f.b[1].position, frame) == PME_OK);
    CHECK(YMPME_Joint_CacheInit(&cache, &entry, 1) == PME_OK);
    f.w.joints = &j; f.w.jointCount = 1; f.w.jointCache = &cache;
    for (int i = 0; i < 240; ++i) step(&f, 1.f/240);
    CHECK(cache.hits == 1 && f.cache.hits > 0);
    pme_vec3 force = j.feedback.reactionB.force;
    for (size_t i = 0; i < f.w.contactCount; ++i)
        force = YMPME_VecAdd(force, f.contacts[i].reactionB.force);
    CHECK(fabsf(force.y - 9.81f) < .01f);
    /* Neither cache, bodies nor joint may advance when either buffer is too small. */
    for (int which = 0; which < 2; ++which)
    {
        if (!which) { cache.count = 0; cache.capacity = 0; }
        else { cache.capacity = 1; f.cache.count = 0; f.cache.capacity = 0; }
        fixture saved = f;
        savedJoint = j; savedCache = cache; savedEntry = entry;
        CHECK(YMPME_Dynamics_Step(&f.w, 1.f/240, f.w.time + 1./240) == PME_CAPACITY_EXCEEDED);
        CHECK(!memcmp(f.b, saved.b, sizeof f.b) && !memcmp(&j, &savedJoint, sizeof j));
        CHECK(!memcmp(&cache, &savedCache, sizeof cache) && !memcmp(&entry, &savedEntry, sizeof entry));
        CHECK(!memcmp(&f.cache, &saved.cache, sizeof f.cache));
        CHECK(!memcmp(f.entries, saved.entries, sizeof f.entries) && f.w.time == saved.w.time);
    }
    f.cache.capacity = 64;
    YMPME_Joint_CacheClear(&cache); YMPME_Dynamics_CacheClear(&f.cache);
    step(&f, 1.f/240); CHECK(cache.hits == 0 && f.cache.hits == 0);
    /* A joint cache can also operate with cold contacts. */
    f.w.cache = NULL;
    step(&f, 1.f/240); CHECK(cache.hits == 1);
}
int main(void)
{
    joint_contact_cache();
	compound();
	transformed_geometry();
	cache_lifecycle();
	float cold = stack(0), warm = stack(1);
	CHECK(warm < cold * .5f);
	puts("DYNAMICS EXTENSIONS PASS: local poses, COM torque, cache lifecycle, timestep/friction, "
		 "low-iteration stack");
	return 0;
}
