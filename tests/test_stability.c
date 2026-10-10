#include "YMPME.h"
#include <float.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define V YMPME_Vec3
#define Q YMPME_Quaternion_Identity()
#define CHECK(x)                                                               \
  do {                                                                         \
    if (!(x)) {                                                                \
      fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #x);                  \
      exit(1);                                                                 \
    }                                                                          \
  } while (0)
#define DT (1.f / 240)
typedef struct fixture_ {
  pme_rigid_body bodies[5];
  pme_collider colliders[5];
  pme_dynamic_contact contacts[32];
  pme_contact_cache cache;
  pme_contact_cache_entry entries[32];
  pme_dynamics world;
} fixture;
static void init(fixture *f) {
  memset(f, 0, sizeof(*f));
  CHECK(YMPME_Rigid_Init(&f->bodies[0], 0, V(0, 0, 0), V(0, 0, 0), Q) ==
        PME_OK);
  f->colliders[0] = YMPME_Dynamics_Collider(0, PME_COLLIDER_PLANE);
  for (int i = 1; i < 5; ++i) {
    CHECK(YMPME_Rigid_Init(&f->bodies[i], 1, V(1.f / 6, 1.f / 6, 1.f / 6),
                           V(0, i - .5f, 0), Q) == PME_OK);
    f->colliders[i] = YMPME_Dynamics_Collider(i, PME_COLLIDER_BOX);
  }
  f->world = YMPME_Dynamics_Default();
  f->world.bodies = f->bodies;
  f->world.bodyCount = 5;
  f->world.colliders = f->colliders;
  f->world.colliderCount = 5;
  f->world.contacts = f->contacts;
  f->world.contactCapacity = 32;
  f->world.iterations = 16;
  CHECK(YMPME_Dynamics_CacheInit(&f->cache, f->entries, 32) == PME_OK);
  f->world.cache = &f->cache;
}
static void finite_state(const pme_rigid_body *b) {
  CHECK(YMPME_Rigid_IsValid(b));
  CHECK(YMPME_VecFinite(b->feedback.acceleration));
  CHECK(YMPME_VecFinite(b->feedback.angularAcceleration));
}
static void stack(void) {
  static fixture a, b;
  init(&a);
  init(&b);
  float drift = 0, speed = 0;
  double t = 0;
  for (int k = 0; t < 60; ++k) {
    /* The ratio stays in [0.5,2]; exercises cache scaling at 120/240/480 Hz. */
    const float dt[] = {DT, 2 * DT, DT, DT * .5f};
    float h = dt[(k / 240) % 4];
    t += (double)h;
    CHECK(YMPME_Dynamics_Step(&a.world, h, t) == PME_OK);
    CHECK(YMPME_Dynamics_Step(&b.world, h, t) == PME_OK);
    CHECK(!memcmp(a.bodies, b.bodies, sizeof a.bodies));
    CHECK(a.world.contactCount == b.world.contactCount);
    for (int i = 1; i < 5; ++i) {
      finite_state(&a.bodies[i]);
      float d = fabsf(a.bodies[i].position.y - (i - .5f));
      drift = fmaxf(drift, d);
      speed = fmaxf(speed, sqrtf(YMPME_VecDot(a.bodies[i].velocity,
                                              a.bodies[i].velocity)));
      CHECK(fabsf(a.bodies[i].position.x) < .01f &&
            fabsf(a.bodies[i].position.z) < .01f);
    }
    CHECK(drift < .02f && speed < .25f);
  }
  CHECK(a.cache.hits > 0);
  /* Teleport/reset the same caller buffers; cleared old impulses must not leak.
   */
  YMPME_Dynamics_CacheClear(&a.cache);
  for (int i = 1; i < 5; ++i) {
    a.bodies[i].position.y += 10;
    a.bodies[i].position.x = 2.f * i;
    a.bodies[i].velocity = V(0, 0, 0);
  }
  CHECK(YMPME_Dynamics_Step(&a.world, DT, t + DT) == PME_OK);
  CHECK(a.world.contactCount == 0 && a.cache.hits == 0);
  for (int i = 1; i < 5; ++i)
    CHECK(fabsf(a.bodies[i].velocity.y + 9.81f * DT) < 1e-5f);
  printf("60 s stack: max height error %.6f m, max speed %.6f m/s; repeated "
         "run identical\n",
         drift, speed);
}
static void joints(void) {
  pme_rigid_body bodies[8];
  pme_joint joints[8];
  pme_joint_cache cache;
  pme_joint_cache_entry entries[8];
  CHECK(YMPME_Joint_CacheInit(&cache, entries, 8) == PME_OK);
  for (int i = 0; i < 8; ++i) {
    CHECK(YMPME_Rigid_Init(&bodies[i], 1, V(.1f, .1f, .1f), V(0, -(float)i, 0),
                           Q) == PME_OK);
    CHECK(YMPME_Joint_Init(&joints[i], PME_JOINT_REVOLUTE, bodies, 8,
                           i ? (size_t)i - 1 : PME_JOINT_WORLD, i,
                           V(0, .5f - i, 0), Q) == PME_OK);
  }
  float error = 0;
  double t = 0;
  for (int k = 0; k < 60 * 240; ++k) {
    t += (double)DT;
    CHECK(YMPME_Joint_StepCached(bodies, 8, joints, 8, V(0, -9.81f, 0), DT, 4,
                                 t, &cache) == PME_OK);
    for (int i = 0; i < 8; ++i) {
      finite_state(&bodies[i]);
      CHECK(isfinite(joints[i].feedback.anchorError));
      error = fmaxf(error, joints[i].feedback.anchorError);
      CHECK(error < .003f);
    }
  }
  CHECK(cache.hits == 8);
  printf("60 s eight-link chain, 4 iterations: max anchor error %.6f m\n",
         error);
}
static void boundaries(void) {
  fixture f;
  init(&f);
  CHECK(YMPME_Dynamics_Step(&f.world, DT, DT) == PME_OK);
  pme_rigid_body saved[5];
  memcpy(saved, f.bodies, sizeof saved);
  pme_contact_cache cache = f.cache;
  pme_contact_cache_entry entries[32];
  memcpy(entries, f.entries, sizeof entries);
  const float tiny = nextafterf(0, 1);
  const float bad[] = {0, -DT, NAN, INFINITY, tiny};
  for (size_t i = 0; i < sizeof bad / sizeof *bad; ++i) {
    CHECK(YMPME_Dynamics_Step(&f.world, bad[i], 2 * DT) ==
          PME_INVALID_ARGUMENT);
    CHECK(!memcmp(saved, f.bodies, sizeof saved));
    CHECK(!memcmp(&cache, &f.cache, sizeof cache));
    CHECK(!memcmp(entries, f.entries, sizeof entries));
    CHECK(f.world.time == (double)DT);
  }
  f.world.time = 1e30;
  for (int i = 0; i < 5; ++i)
    f.bodies[i].feedback.time = 1e30;
  memcpy(saved, f.bodies, sizeof saved);
  CHECK(YMPME_Dynamics_Step(&f.world, DT, 1e30 + DT) == PME_INVALID_ARGUMENT);
  CHECK(!memcmp(saved, f.bodies, sizeof saved));
  CHECK(YMPME_Joint_Step(f.bodies, 5, NULL, 0, V(0, 0, 0), DT, 4, 1e30 + DT) ==
        PME_INVALID_ARGUMENT);
  CHECK(!memcmp(saved, f.bodies, sizeof saved));
  CHECK(YMPME_Rigid_Step(&f.bodies[1], V(0, 0, 0), DT, 1e30 + DT) ==
        PME_INVALID_ARGUMENT);
  CHECK(!memcmp(saved, f.bodies, sizeof saved));
  pme_rigid_body body;
  CHECK(YMPME_Rigid_Init(&body, .01f, V(.1f, .1f, .1f), V(0, 0, 0), Q) ==
        PME_OK);
  body.load.force.x = FLT_MAX;
  pme_rigid_body before = body;
  CHECK(YMPME_Rigid_Step(&body, V(0, 0, 0), .05f, .05) == PME_INVALID_ARGUMENT);
  CHECK(!memcmp(&before, &body, sizeof body));
  body.load.force.x = 0;
  CHECK(YMPME_Rigid_Step(&body, V(0, 0, 0), tiny, DT) == PME_INVALID_ARGUMENT);
  pme_world core;
  pme_body storage[1];
  pme_world_desc desc = YMPME_World_DefaultDesc();
  desc.fixedStep = tiny;
  CHECK(YMPME_World_Init(&core, storage, 1, &desc) == PME_INVALID_ARGUMENT);
  desc.fixedStep = DT;
  CHECK(YMPME_World_Init(&core, storage, 1, &desc) == PME_OK);
  CHECK(YMPME_World_Step(&core, tiny) == PME_INVALID_ARGUMENT);
  CHECK(core.stepCount == 0);
  CHECK(YMPME_World_Destroy(&core) == PME_OK);
  puts("Unrepresentable dt/time and free-body overflow rejected without state "
       "commit");
}
int main(int argc, char **argv) {
  CHECK(argc == 2);
  if (!strcmp(argv[1], "stack"))
    stack();
  else if (!strcmp(argv[1], "joints"))
    joints();
  else if (!strcmp(argv[1], "boundaries"))
    boundaries();
  else
    CHECK(0);
  return 0;
}
