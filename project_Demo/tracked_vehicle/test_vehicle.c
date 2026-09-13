#include "vehicle.h"
#include <stdio.h>
#include <string.h>
#define CHECK(x)                                                                                   \
    do {                                                                                           \
        if (!(x)) {                                                                                \
            fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #x);                                   \
            return 0;                                                                              \
        }                                                                                          \
    } while (0)
static tracked_vehicle s;
static float maxError, maxDepth, maxLateral, maxWall;
static int initialize(int scenario) {
    maxError = maxDepth = maxLateral = maxWall = 0;
    CHECK(TV_Init(&s, scenario));
    CHECK(s.world.jointCount == TV_JOINTS && s.world.bodyCount == TV_BODIES);
    for (int side = 0; side < 2; ++side)
        for (int i = 0; i < TV_LINKS_PER_SIDE; ++i) {
            const pme_joint *j = &s.joints[side * TV_LINKS_PER_SIDE + i];
            CHECK(j->type == PME_JOINT_REVOLUTE);
            CHECK(j->bodyA == (size_t)(TV_LINK_FIRST + side * TV_LINKS_PER_SIDE + i));
            CHECK(j->bodyB == (size_t)(TV_LINK_FIRST + side * TV_LINKS_PER_SIDE + (i + 1) % TV_LINKS_PER_SIDE));
        }
    return 1;
}
static int step(float throttle, float steer) {
    CHECK(TV_Step(&s, throttle, steer, 0));
    maxError = fmaxf(maxError, s.anchorError);
    maxDepth = fmaxf(maxDepth, s.penetration);
    maxLateral = fmaxf(maxLateral, s.lateralError);
    maxWall = fmaxf(maxWall, fabsf(s.wallLoad.torque.y));
    CHECK(s.anchorError < .012f);
    CHECK(s.penetration < .015f);
    CHECK(s.lateralError < .012f);
    CHECK(fabsf(s.motorTorque[0]) <= 60.01f && fabsf(s.motorTorque[1]) <= 60.01f);
    return 1;
}
static int advance(float seconds, float throttle, float steer) {
    for (int i = 0; i < (int)(seconds / TV_DT); ++i)
        CHECK(step(throttle, steer));
    return 1;
}
static pme_vec3 centerOfMass(void) {
    pme_vec3 p = YMPME_Vec3(0, 0, 0);
    for (int i = 0; i < TV_BODIES; ++i)
        p = YMPME_VecAdd(p, YMPME_VecScale(s.bodies[i].position, s.bodies[i].mass / s.mass));
    return p;
}
static pme_vec3 localMarker(void) {
    const pme_rigid_body *frame = &s.bodies[TV_BOGIE_FIRST];
    return YMPME_Rotation_InverseApply(
        YMPME_Quaternion_Rotation(frame->orientation),
        YMPME_VecSub(s.bodies[TV_LINK_FIRST].position, frame->position));
}
/* Check all link boxes, not only a set of support points. The same box
   geometry and pose is also checked against the rendered mesh by the GUI test. */
static float vertexDepth(void) {
    float worst = 0;
    for (int id = TV_LINK_FIRST; id < TV_LINK_FIRST + TV_LINKS; ++id) {
        const pme_rigid_body *b = &s.bodies[id];
        pme_vec3 h = s.colliders[id].halfSize;
        pme_rotation r = YMPME_Quaternion_Rotation(b->orientation);
        for (int k = 0; k < 8; ++k) {
            pme_vec3 p =
                YMPME_VecAdd(b->position, YMPME_Rotation_Apply(r, YMPME_Vec3(k & 1 ? h.x : -h.x,
                                                                             k & 2 ? h.y : -h.y,
                                                                             k & 4 ? h.z : -h.z)));
            worst = fmaxf(worst, -p.y);
            for (int i = 5; i < TV_STATIC_END; ++i) {
                pme_vec3 d = YMPME_VecSub(p, s.bodies[i].position), a = s.colliders[i].halfSize;
                worst = fmaxf(worst,
                              fminf(a.x - fabsf(d.x), fminf(a.y - fabsf(d.y), a.z - fabsf(d.z))));
            }
        }
    }
    return worst;
}
static int test(const char *name) {
    if (!strcmp(name, "forward") || !strcmp(name, "reverse")) {
        CHECK(initialize(0));
        pme_vec3 marker = localMarker();
        float direction = !strcmp(name, "forward") ? 1 : -1;
        CHECK(advance(3, direction, 0));
        CHECK(direction * (s.bodies[0].position.z + 3) > .8f);
        CHECK(direction * s.wheelSpeed[0] > .7f && direction * s.wheelSpeed[1] > .7f);
        CHECK(s.beltSpeed[0] > .3f && s.beltSpeed[1] > .3f);
        pme_vec3 moved = YMPME_VecSub(marker, localMarker());
        CHECK(YMPME_VecDot(moved, moved) > .1f);
        s.motor = 0;
        CHECK(advance(.2f, direction, 0));
        CHECK(s.motorTorque[0] == 0 && s.motorTorque[1] == 0);
    } else if (!strcmp(name, "turn")) {
        CHECK(initialize(0));
        CHECK(advance(4.5f, 0, 1));
        CHECK(TV_Yaw(&s) > .5f);
        CHECK(s.wheelSpeed[0] > .8f && s.wheelSpeed[1] < -.8f);
    } else if (!strcmp(name, "wall")) {
        CHECK(initialize(1));
        CHECK(advance(6.5f, 1, 0));
        CHECK(fabsf(TV_Yaw(&s)) < .08f);
        CHECK(maxWall > 20);
        /* Check the actual chassis front against the wall, independent of vehicle length. */
        pme_rotation r = YMPME_Quaternion_Rotation(s.bodies[0].orientation);
        pme_vec3 h = s.colliders[0].halfSize;
        float front = s.bodies[0].position.z + fabsf(r.axis[0].z) * h.x +
                      fabsf(r.axis[1].z) * h.y + fabsf(r.axis[2].z) * h.z;
        CHECK(front < s.bodies[2].position.z - s.colliders[2].halfSize.z + .012f);
    } else if (!strcmp(name, "terrain")) {
        CHECK(initialize(3));
        float deepest = 0;
        size_t contacts = 0;
        for (int i = 0; i < TV_HZ * 3; ++i) {
            CHECK(step(0, i > TV_HZ * 2 ? .5f : 0));
            if (i % 4 == 0)
                deepest = fmaxf(deepest, vertexDepth());
            for (size_t j = 0; j < s.world.contactCount; ++j) {
                const pme_dynamic_contact *c = &s.contacts[j];
                if (c->normalImpulse > 0 &&
                    ((TV_LinkSide(c->bodyA) >= 0 && c->bodyB >= 5 && c->bodyB < TV_STATIC_END) ||
                     (TV_LinkSide(c->bodyB) >= 0 && c->bodyA >= 5 && c->bodyA < TV_STATIC_END)))
                    ++contacts;
            }
        }
        printf("terrain: actual link contacts %zu, vertex depth %.3f mm\n", contacts,
               deepest * 1000);
        CHECK(contacts > 100);
        CHECK(deepest < .012f);
    } else if (!strcmp(name, "friction")) {
        CHECK(initialize(0));
        s.friction = 0;
        pme_vec3 first = centerOfMass();
        CHECK(advance(3, 1, 0));
        pme_vec3 moved = YMPME_VecSub(centerOfMass(), first);
        printf("zero friction COM displacement %.6f %.6f m, moving links %.3f %.3f m/s\n", moved.x,
               moved.z, s.beltSpeed[0], s.beltSpeed[1]);
        CHECK(hypotf(moved.x, moved.z) < .003f);
        CHECK(s.beltSpeed[0] > .3f && s.beltSpeed[1] > .3f);
    } else
        return 0;
    printf("%s PASS: joint %.3f mm, penetration %.3f mm, axial offset %.3f mm, yaw %.3f rad\n",
           name, maxError * 1000, maxDepth * 1000, maxLateral * 1000, TV_Yaw(&s));
    return 1;
}
int main(int argc, char **argv) {
    if (argc == 2)
        return test(argv[1]) ? 0 : 1;
    if (argc != 1)
        return 1;
    const char *cases[] = {"forward", "reverse", "turn", "wall", "terrain", "friction"};
    for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i)
        if (!test(cases[i]))
            return 1;
    return 0;
}
