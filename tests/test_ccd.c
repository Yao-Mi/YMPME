#include "DYNAMICS/YMPME_Dynamics.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define CHECK(x)                                                                                   \
    do                                                                                             \
    {                                                                                              \
        if (!(x))                                                                                  \
        {                                                                                          \
            fprintf(stderr, "line %d: %s\n", __LINE__, #x);                                        \
            exit(1);                                                                               \
        }                                                                                          \
    } while (0)
#define V YMPME_Vec3
#define Q YMPME_Quaternion_Identity()
#define DT .02f
#define NEAR(a, b, e) (fabsf((a) - (b)) < (e))
typedef struct fixture_
{
    pme_rigid_body b[3];
    pme_collider c[3];
    pme_dynamic_contact contacts[32];
    pme_dynamics w;
    pme_ccd ccd;
    uint8_t enabled[3];
    pme_ccd_event events[32];
} fixture;
static void init(fixture *f, int on)
{
    memset(f, 0, sizeof *f);
    for (int i = 0; i < 3; ++i)
        CHECK(YMPME_Rigid_Init(&f->b[i], i == 1 ? 1 : 0, V(.004f, .004f, .004f), V(0, 0, 0), Q) ==
              PME_OK);
    f->b[1].position = V(-1, 0, 0);
    f->b[1].velocity = V(100, 0, 0);
    f->c[0] = YMPME_Dynamics_Collider(0, PME_COLLIDER_BOX);
    f->c[0].halfSize = V(.01f, 1, 1);
    f->c[1] = YMPME_Dynamics_Collider(1, PME_COLLIDER_SPHERE);
    f->c[1].radius = .1f;
    f->c[2] = YMPME_Dynamics_Collider(2, PME_COLLIDER_BOX);
    f->c[2].halfSize = V(.01f, 1, 1);
    f->b[2].position.x = -2;
    for (int i = 0; i < 3; ++i)
        f->c[i].friction = 0;
    f->w = YMPME_Dynamics_Default();
    f->w.bodies = f->b;
    f->w.bodyCount = 3;
    f->w.colliders = f->c;
    f->w.colliderCount = 2;
    f->w.contacts = f->contacts;
    f->w.contactCapacity = 32;
    f->w.gravity = V(0, 0, 0);
    f->enabled[1] = 1;
    CHECK(YMPME_Dynamics_CCDInit(&f->ccd, f->enabled, 3, f->events, 32) == PME_OK);
    if (on)
        f->w.ccd = &f->ccd;
}
static void step(fixture *f) { CHECK(YMPME_Dynamics_Step(&f->w, DT, f->w.time + DT) == PME_OK); }
static void wall(void)
{
    fixture f;
    init(&f, 0);
    step(&f);
    CHECK(f.b[1].position.x > .9f);
    init(&f, 1);
    step(&f);
    CHECK(NEAR(f.b[1].position.x, -.11f, 1e-5f));
    CHECK(f.ccd.count == 1);
    CHECK(NEAR(f.events[0].time, .0089f, 1e-6f));
    CHECK(f.b[1].velocity.x == 0);
    CHECK(NEAR(f.events[0].normalImpulse, 100, 1e-4f));
    CHECK(NEAR(f.events[0].reactionB.force.x, f.b[1].feedback.acceleration.x, 1e-3f));
    CHECK(f.w.contactCount == 0 && f.b[1].feedback.time == f.w.time);
    for (int i = 0; i < 100; ++i)
        step(&f);
    CHECK(NEAR(f.b[1].position.x, -.11f, 1e-5f));
    init(&f, 1);
    f.c[0].restitution = f.c[1].restitution = 1;
    step(&f);
    CHECK(NEAR(f.b[1].position.x, -1.22f, 2e-5f));
    CHECK(NEAR(f.b[1].velocity.x, -100, 1e-5f));
    /* Gravity/loads change this step's swept velocity before TOI search. */
    init(&f, 1);
    f.b[1].velocity = V(0, 0, 0);
    f.b[1].load.force = V(5000, 0, 0);
    step(&f);
    CHECK(f.ccd.count == 1 && f.b[1].position.x < 0 && f.b[1].load.force.x == 0);
}
static void planes_filters(void)
{
    fixture f;
    init(&f, 1);
    f.c[0].type = PME_COLLIDER_PLANE;
    f.b[1].position = V(0, 1, 0);
    f.b[1].velocity = V(0, -100, 0);
    step(&f);
    CHECK(f.ccd.count == 1 && NEAR(f.b[1].position.y, .1f, 1e-5f));
    init(&f, 1);
    f.c[0].mask = 0;
    step(&f);
    CHECK(f.ccd.count == 0 && f.b[1].position.x > .9f);
    init(&f, 1);
    f.c[0].group = f.c[1].group = -3;
    step(&f);
    CHECK(f.ccd.count == 0);
    init(&f, 1);
    f.c[0].mask = 0;
    f.c[0].group = f.c[1].group = 3;
    step(&f);
    CHECK(f.ccd.count == 1);
    /* Different dynamic bodies remain discrete, even when a sphere opts into static CCD. */
    init(&f, 1);
    CHECK(YMPME_Rigid_Init(&f.b[0], 1, V(1, 1, 1), V(0, 0, 0), Q) == PME_OK);
    step(&f);
    CHECK(f.ccd.count == 0);
}
static void multiple(void)
{
    fixture f;
    init(&f, 1);
    f.w.colliderCount = 3;
    for (int i = 0; i < 3; ++i)
        f.c[i].restitution = 1;
    f.b[1].velocity.x = 1000;
    step(&f);
    CHECK(f.ccd.count == 8 && f.ccd.clampedBodies == 1);
    CHECK(f.b[1].position.x >= -1.891f && f.b[1].position.x <= -.109f);
    init(&f, 1);
    f.w.colliderCount = 3;
    for (int i = 0; i < 3; ++i)
        f.c[i].restitution = 1;
    f.b[1].velocity.x = 200;
    step(&f);
    CHECK(f.ccd.count == 2 && f.ccd.clampedBodies == 0);
    CHECK(NEAR(f.b[1].position.x, -.56f, 1e-4f));
    CHECK(NEAR(f.b[1].velocity.x, 200, 1e-4f));
    CHECK(f.events[1].time > f.events[0].time);
}
static void rounded(void)
{
    fixture f;
    init(&f, 1);
    f.c[0].halfSize = V(.01f, .5f, .5f);
    f.b[1].position = V(-1, .58f, .58f);
    step(&f);
    CHECK(f.ccd.count == 0); /* inside expanded AABB, outside rounded corner */
    init(&f, 1);
    f.c[0].halfSize = V(.01f, .5f, .5f);
    f.b[1].position = V(-1, .56f, .56f);
    step(&f);
    CHECK(f.ccd.count == 1);
    CHECK(f.events[0].normal.y > .5f && f.events[0].normal.z > .5f);
    CHECK(NEAR(f.events[0].time, (1 - .01f - sqrtf(.01f - .0072f)) / 100, 2e-6f));
    /* Capsule edge and exact face grazing. */
    init(&f, 1);
    f.c[0].halfSize = V(.01f, .5f, .5f);
    f.b[1].position = V(-1, .56f, 0);
    step(&f);
    CHECK(f.ccd.count >= 1 && NEAR(f.events[0].time, .0091f, 2e-6f));
}
static void transforms_mesh_friction(void)
{
    fixture f;
    init(&f, 1);
    pme_quaternion q;
    CHECK(YMPME_Quaternion_AxisAngle(V(0, 0, 1), .4f, &q) == PME_OK);
    f.c[0].localOrientation = q;
    f.c[0].localPosition = V(.2f, .1f, 0);
    pme_vec3 n = YMPME_Rotation_Apply(YMPME_Quaternion_Rotation(q), V(1, 0, 0));
    f.b[1].position = YMPME_VecSub(f.c[0].localPosition, n);
    f.b[1].velocity = YMPME_VecScale(n, 100);
    step(&f);
    CHECK(f.ccd.count == 1 && NEAR(f.events[0].time, .0089f, 1e-6f));
    pme_vec3 vertices[8], fa[12], ea[36];
    pme_convex_mesh mesh;
    for (int i = 0; i < 8; ++i)
        vertices[i] = V((i & 1 ? 1 : -1) * .01f, (i & 2 ? 1 : -1), (i & 4 ? 1 : -1));
    PME_Index indices[36] = {0, 2, 3, 0, 3, 1, 4, 5, 7, 4, 7, 6, 0, 1, 5, 0, 5, 4,
                            2, 6, 7, 2, 7, 3, 0, 4, 6, 0, 6, 2, 1, 3, 7, 1, 7, 5};
    CHECK(YMPME_ConvexMesh_Init(&mesh, vertices, 8, indices, 12, fa, 12, ea, 36) == PME_OK);
    init(&f, 1);
    f.c[0].type = PME_COLLIDER_MESH;
    f.c[0].mesh = &mesh;
    step(&f);
    CHECK(f.ccd.count == 1 && NEAR(f.events[0].time, .0089f, 1e-6f));
    init(&f, 1);
    f.c[0].friction = f.c[1].friction = .5f;
    f.b[1].velocity.y = 10;
    step(&f);
    CHECK(f.ccd.count == 1 && f.b[1].velocity.y < 10 && fabsf(f.b[1].angularVelocity.z) > 1);
    CHECK(fabsf(f.events[0].tangentImpulse.y) <= .5f * f.events[0].normalImpulse);
    CHECK(
        NEAR(f.events[0].reactionB.torque.z, .004f * f.b[1].feedback.angularAcceleration.z, .001f));
}
static void invalid(void)
{
    fixture f;
    init(&f, 1);
    f.ccd.capacity = 7;
    fixture saved = f;
    CHECK(YMPME_Dynamics_Step(&f.w, DT, DT) == PME_CAPACITY_EXCEEDED);
    CHECK(!memcmp(f.b, saved.b, sizeof f.b) && !memcmp(&f.ccd, &saved.ccd, sizeof f.ccd));
    CHECK(f.w.time == 0);
    init(&f, 1);
    f.c[1].localPosition.x = .1f;
    saved = f;
    CHECK(YMPME_Dynamics_Step(&f.w, DT, DT) == PME_INVALID_ARGUMENT);
    CHECK(!memcmp(f.b, saved.b, sizeof f.b));
    init(&f, 1);
    pme_joint j;
    CHECK(YMPME_Joint_Init(&j, PME_JOINT_FIXED, f.b, 3, PME_JOINT_WORLD, 1, f.b[1].position, Q) ==
          PME_OK);
    f.w.joints = &j;
    f.w.jointCount = 1;
    CHECK(YMPME_Dynamics_Step(&f.w, DT, DT) == PME_INVALID_ARGUMENT);
    init(&f, 1);
    f.enabled[1] = 0;
    f.ccd.capacity = 0;
    step(&f);
    CHECK(f.b[1].position.x > .9f && f.ccd.count == 0);
}
static void cache_separation(void)
{
    fixture f;
    init(&f, 1);
    pme_contact_cache cache;
    pme_contact_cache_entry entries[32];
    CHECK(YMPME_Dynamics_CacheInit(&cache,entries,32)==PME_OK);
    f.w.cache=&cache;
    f.w.colliderCount=3;
    f.c[2].type=PME_COLLIDER_PLANE;
    f.b[2].position=V(0,-.1f,0);
    step(&f);
    CHECK(f.ccd.count==1 && cache.count==1 && f.w.contactCount==1);
    CHECK(f.contacts[0].colliderA==2 && f.events[0].colliderA==0);
    step(&f);
    CHECK(f.ccd.count==0);
    /* First impact moved the ground anchor by .89 m; only the following resting step matches. */
    step(&f);
    CHECK(f.ccd.count==0 && cache.hits==1);
    /* Existing cache and CCD output both survive a rejected step. */
    pme_contact_cache saved=cache;
    pme_contact_cache_entry savedEntries[32];memcpy(savedEntries,entries,sizeof entries);
    fixture old=f;f.ccd.capacity=7;
    CHECK(YMPME_Dynamics_Step(&f.w,DT,f.w.time+DT)==PME_CAPACITY_EXCEEDED);
    CHECK(!memcmp(&cache,&saved,sizeof cache) && !memcmp(entries,savedEntries,sizeof entries));
    CHECK(!memcmp(f.events,old.events,sizeof f.events) && !memcmp(f.b,old.b,sizeof f.b));
    CHECK(f.ccd.count==old.ccd.count && f.w.time==old.w.time);
}
int main(void)
{
    cache_separation();
    wall();
    planes_filters();
    multiple();
    rounded();
    transforms_mesh_friction();
    invalid();
    puts("CCD PASS: thin wall, TOI/rebound, plane/convex/local pose, edge/corner, force/spin "
         "feedback, filtering, bounded impacts, validation");
    return 0;
}
