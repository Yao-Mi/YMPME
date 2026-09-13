#include "JOINT/YMPME_Joint.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define CHECK(x)                                                                                   \
    do {                                                                                           \
        if (!(x)) {                                                                                \
            fprintf(stderr, "line %d: %s\n", __LINE__, #x);                                        \
            exit(1);                                                                               \
        }                                                                                          \
    } while (0)
#define V(x, y, z) YMPME_Vec3(x, y, z)
#define Q YMPME_Quaternion_Identity()
#define DT (1.f / 240)
#define NEAR(x, y, e) (fabsf((x) - (y)) < (e))
static void tick(pme_rigid_body *b, pme_joint *j, pme_joint_cache *c, float dt) {
    CHECK(YMPME_Joint_StepCached(b, 1, j, 1, V(0, -9.81f, 0), dt, 4, b->feedback.time + dt, c) ==
          PME_OK);
}
static void support(void) {
    pme_rigid_body b, saved;
    pme_joint j, js;
    pme_joint_cache c, cs;
    pme_joint_cache_entry entries[1], es;
    CHECK(YMPME_Rigid_Init(&b, 2, V(1, 1, 1), V(0, 0, 0), Q) == PME_OK);
    CHECK(YMPME_Joint_Init(&j, PME_JOINT_FIXED, &b, 1, PME_JOINT_WORLD, 0, V(0, 0, 0), Q) ==
          PME_OK);
    CHECK(YMPME_Joint_CacheInit(&c, entries, 1) == PME_OK);
    for (int i = 0; i < 240; ++i) {
        tick(&b, &j, &c, DT);
        CHECK(c.hits == (size_t)(i > 0));
        CHECK(NEAR(j.feedback.reactionB.force.y, 19.62f, .002f));
        CHECK(NEAR(b.position.y, 0, 1e-6f));
    }
    tick(&b, &j, &c, DT * 2);
    CHECK(c.hits == 1);
    CHECK(NEAR(j.feedback.reactionB.force.y, 19.62f, .002f));
    tick(&b, &j, &c, DT / 2);
    CHECK(c.hits == 0);
    tick(&b, &j, &c, DT / 2);
    CHECK(c.hits == 1);
    j.enabled = 0;
    tick(&b, &j, &c, DT / 2);
    CHECK(c.hits == 0 && c.entries[0].rowCount == 0);
    CHECK(NEAR(j.feedback.reactionB.force.y, 0, 1e-6f));
    j.enabled = 1;
    tick(&b, &j, &c, DT / 2);
    CHECK(c.hits == 0);
    tick(&b, &j, &c, DT / 2);
    CHECK(c.hits == 1);
    j.localAnchorA.x += .001f;
    tick(&b, &j, &c, DT / 2);
    CHECK(c.hits == 0);
    YMPME_Joint_CacheClear(&c);
    tick(&b, &j, &c, DT / 2);
    CHECK(c.hits == 0);
    /* Capacity and corrupt-state failures are transactional. */
    c.capacity = 0;
    c.count = 0;
    saved = b;
    js = j;
    cs = c;
    es = entries[0];
    CHECK(YMPME_Joint_StepCached(&b, 1, &j, 1, V(0, 0, 0), DT, 4, b.feedback.time + DT, &c) ==
          PME_CAPACITY_EXCEEDED);
    CHECK(!memcmp(&b, &saved, sizeof b) && !memcmp(&j, &js, sizeof j));
    CHECK(!memcmp(&c, &cs, sizeof c) && !memcmp(entries, &es, sizeof es));
    c.capacity = 1;
    c.count = 1;
    entries[0].impulses[0] = NAN;
    saved = b;
    js = j;
    CHECK(YMPME_Joint_StepCached(&b, 1, &j, 1, V(0, 0, 0), DT, 4, b.feedback.time + DT, &c) ==
          PME_INVALID_ARGUMENT);
    CHECK(!memcmp(&b, &saved, sizeof b) && !memcmp(&j, &js, sizeof j));
    YMPME_Joint_CacheClear(&c);
    tick(&b, &j, &c, DT);
    /* Owner changes and time discontinuities must not consume old impulses. */
    saved = b;
    tick(&saved, &j, &c, DT);
    CHECK(c.hits == 0);
    CHECK(YMPME_Joint_StepCached(&saved, 1, &j, 1, V(0, 0, 0), DT, 4, saved.feedback.time + DT * 3,
                                 &c) == PME_OK);
    CHECK(c.hits == 0);
}
static void actuators(pme_joint_type type) {
    pme_rigid_body b;
    pme_joint j;
    pme_joint_cache c;
    pme_joint_cache_entry entries[1];
    CHECK(YMPME_Rigid_Init(&b, 1, V(1, 1, 1), V(0, 0, 0), Q) == PME_OK);
    CHECK(YMPME_Joint_Init(&j, type, &b, 1, PME_JOINT_WORLD, 0, V(0, 0, 0), Q) == PME_OK);
    CHECK(YMPME_Joint_CacheInit(&c, entries, 1) == PME_OK);
    CHECK(YMPME_Joint_SetLimit(&j, (pme_joint_limit){1, -.3f, .4f}) == PME_OK);
    for (int sign = 1; sign >= -1; sign -= 2) {
        CHECK(YMPME_Joint_SetMotor(
                  &j, (pme_joint_motor){PME_JOINT_MOTOR_SPEED, sign * 2.f, 0, 10}) == PME_OK);
        for (int i = 0; i < 720; ++i) {
            tick(&b, &j, &c, DT);
            CHECK(fabsf(j.feedback.motorEffort) < 10.001f);
            CHECK(j.coordinate >= -.302f && j.coordinate <= .402f);
        }
        CHECK(NEAR(j.coordinate, sign > 0 ? .4f : -.3f, .002f));
    }
    j.limit = (pme_joint_limit){1, -.5f, .6f};
    tick(&b, &j, &c, DT);
    CHECK(c.rowHits == 6); /* only structural and motor rows, changed limits discarded */
    j.limit.enabled = 0;
    tick(&b, &j, &c, DT);
    CHECK(j.rowCount == 6 && c.rowHits == 6);
    j.motor = (pme_joint_motor){PME_JOINT_MOTOR_EFFORT, 0, 3, 3};
    tick(&b, &j, &c, DT);
    CHECK(c.rowHits == 5 && NEAR(j.feedback.motorEffort, 3, .001f));
    j.motor.effort = -2;
    j.motor.maxEffort = 1;
    tick(&b, &j, &c, DT);
    CHECK(NEAR(j.feedback.motorEffort, -1, .001f));
    j.motor.mode = PME_JOINT_MOTOR_OFF;
    tick(&b, &j, &c, DT);
    CHECK(c.rowHits == 5 && j.feedback.motorEffort == 0);
    j.limit = (pme_joint_limit){1, -.5f, .6f};
    tick(&b, &j, &c, DT);
    CHECK(c.rowHits == 5);
    tick(&b, &j, &c, DT);
    CHECK(c.rowHits == 7);
    CHECK(YMPME_Quaternion_AxisAngle(V(0, 1, 0), .1f, &j.localFrameA) == PME_OK);
    tick(&b, &j, &c, DT);
    CHECK(c.hits == 0);
}
static float chain(int warm) {
    pme_rigid_body b[8];
    pme_joint j[8];
    pme_joint_cache c;
    pme_joint_cache_entry entries[8];
    CHECK(YMPME_Joint_CacheInit(&c, entries, 8) == PME_OK);
    for (int i = 0; i < 8; ++i) {
        CHECK(YMPME_Rigid_Init(&b[i], 1, V(.1f, .1f, .1f), V(0, -(float)i, 0), Q) == PME_OK);
        CHECK(YMPME_Joint_Init(&j[i], PME_JOINT_REVOLUTE, b, 8,
                               i ? ((size_t)i - 1) : PME_JOINT_WORLD, i, V(0, .5f - i, 0),
                               Q) == PME_OK);
    }
    float error = 0;
    for (int k = 0; k < 2400; ++k) {
        CHECK(YMPME_Joint_StepCached(b, 8, j, 8, V(0, -9.81f, 0), DT, 4, (k + 1) * (double)DT,
                                     warm ? &c : NULL) == PME_OK);
        if (k >= 1200)
            for (int i = 0; i < 8; ++i)
                error = fmaxf(error, j[i].feedback.anchorError);
    }
    CHECK(!warm || c.hits == 8);
    return error;
}
int main(void) {
    support();
    actuators(PME_JOINT_REVOLUTE);
    actuators(PME_JOINT_PRISMATIC);
    float cold = chain(0), warm = chain(1);
    printf("8-link chain, 4 iterations: cold %.6f m, warm %.6f m\n", cold, warm);
    CHECK(warm < cold * .5f);
    puts("joint warm-start checks passed");
    return 0;
}
