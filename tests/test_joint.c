#include "JOINT/YMPME_Joint.h"
#include <stdio.h>
#include <string.h>
#define CHECK(c)                                                                                   \
    do                                                                                             \
    {                                                                                              \
        if (!(c))                                                                                  \
        {                                                                                          \
            fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #c);                                \
            return 1;                                                                              \
        }                                                                                          \
    } while (0)
#define NEAR(a, b, t) (fabsf((a) - (b)) < (t))
#define V(x, y, z) YMPME_Vec3(x, y, z)
#define Q YMPME_Quaternion_Identity()
static float norm(pme_vec3 v) { return sqrtf(YMPME_VecDot(v, v)); }
static int step(pme_rigid_body *b, size_t n, pme_joint *j, size_t nj, pme_vec3 g, int count)
{
    int i;
    double t = b[0].feedback.time;
    for (i = 0; i < count; ++i)
    {
        t += 1.0 / 240;
        if (YMPME_Joint_Step(b, n, j, nj, g, 1.f / 240, 64, t) != PME_OK)
            return 0;
    }
    return 1;
}
int main(void)
{
    pme_rigid_body b[4], saved;
    pme_joint j[4], js;
    pme_quaternion frame;
    pme_vec3 momentum;
    int i;
    float c;
    CHECK(YMPME_Rigid_Init(&b[0], 1, V(1, 1, 1), V(1, 0, 0), Q) == PME_OK);
    CHECK(YMPME_Joint_Init(&j[0], PME_JOINT_FIXED, b, 1, PME_JOINT_WORLD, 0, V(0, 0, 0), Q) ==
          PME_OK);
    CHECK(step(b, 1, j, 1, V(0, -9.81f, 0), 240));
    CHECK(NEAR(b[0].position.x, 1, 1e-5f) && NEAR(b[0].position.y, 0, 1e-5f));
    CHECK(NEAR(j[0].feedback.reactionB.force.y, 9.81f, .002f));
    CHECK(NEAR(j[0].feedback.reactionA.torque.z, -9.81f, .002f));
    CHECK(norm(j[0].feedback.reactionB.torque) < .002f);
    /* Dynamic-dynamic motor transfers angular momentum, and acts in 3D. */
    CHECK(YMPME_Quaternion_AxisAngle(V(0, 1, 0), 1.570796327f, &frame) == PME_OK);
    for (i = 0; i < 2; ++i)
        CHECK(YMPME_Rigid_Init(&b[i], 1, V(1, 1, 1), V(0, 0, 0), Q) == PME_OK);
    CHECK(YMPME_Joint_Init(j, PME_JOINT_REVOLUTE, b, 2, 0, 1, V(0, 0, 0), frame) == PME_OK);
    CHECK(YMPME_Joint_SetMotor(j, (pme_joint_motor){PME_JOINT_MOTOR_EFFORT, 0, 2, 2}) == PME_OK);
    CHECK(step(b, 2, j, 1, V(0, 0, 0), 240));
    CHECK(NEAR(b[0].angularVelocity.x, -2, .001f) && NEAR(b[1].angularVelocity.x, 2, .001f));
    CHECK(NEAR(j->feedback.motorEffort, 2, 1e-5f));
    CHECK(norm(YMPME_VecAdd(j->feedback.reactionA.torque, j->feedback.reactionB.torque)) < 1e-5f);
    CHECK(YMPME_Joint_SetMotor(j, (pme_joint_motor){PME_JOINT_MOTOR_SPEED, 4, 0, 5}) == PME_OK);
    CHECK(step(b, 2, j, 1, V(0, 0, 0), 720));
    CHECK(j->feedback.coordinate > 12 && NEAR(j->feedback.speed, 4, .005f));
    CHECK(YMPME_Joint_SetMotor(j, (pme_joint_motor){PME_JOINT_MOTOR_SPEED, -4, 0, 5}) == PME_OK);
    CHECK(step(b, 2, j, 1, V(0, 0, 0), 1440));
    CHECK(j->feedback.coordinate < 0);
    /* Limits stop a powered hinge at either end, without reversing the body by teleport. */
    for (i = 0; i < 2; ++i)
        CHECK(YMPME_Rigid_Init(&b[i], i ? 1 : 0, V(1, 1, 1), V(0, 0, 0), Q) == PME_OK);
    CHECK(YMPME_Joint_Init(j, PME_JOINT_REVOLUTE, b, 2, 0, 1, V(0, 0, 0), Q) == PME_OK);
    CHECK(YMPME_Joint_SetLimit(j, (pme_joint_limit){1, -.5f, .6f}) == PME_OK);
    CHECK(YMPME_Joint_SetMotor(j, (pme_joint_motor){PME_JOINT_MOTOR_SPEED, 8, 0, 10}) == PME_OK);
    CHECK(step(b, 2, j, 1, V(0, 0, 0), 480));
    CHECK(NEAR(j->feedback.coordinate, .6f, .005f));
    j->motor.speed = -8;
    CHECK(step(b, 2, j, 1, V(0, 0, 0), 480));
    CHECK(NEAR(j->feedback.coordinate, -.5f, .005f));
    /* Slider constrains five DOFs, uses SI forces, and supports a load. */
    CHECK(YMPME_Rigid_Init(b, 2, V(1, 2, 3), V(0, 0, 0), Q) == PME_OK);
    CHECK(YMPME_Joint_Init(j, PME_JOINT_PRISMATIC, b, 1, PME_JOINT_WORLD, 0, V(0, 0, 0), frame) ==
          PME_OK);
    CHECK(YMPME_Joint_SetMotor(j, (pme_joint_motor){PME_JOINT_MOTOR_SPEED, 1, 0, 4}) == PME_OK);
    CHECK(YMPME_Joint_SetLimit(j, (pme_joint_limit){1, -1, 1}) == PME_OK);
    CHECK(step(b, 1, j, 1, V(0, -9.81f, 0), 480));
    CHECK(NEAR(b[0].position.x, 1, .005f) && fabsf(b[0].position.y) < 1e-5f);
    CHECK(NEAR(j->feedback.reactionB.force.y, 19.62f, .005f));
    CHECK(fabsf(j->feedback.motorEffort) <= 4.001f && j->feedback.angularError < 1e-5f);
    /* A moving reference frame: offset slider Jacobian conserves total angular momentum. */
    for (i = 0; i < 2; ++i)
        CHECK(YMPME_Rigid_Init(&b[i], 1, V(1, 1, 1), V((float)i, 0, 0), Q) == PME_OK);
    CHECK(YMPME_Joint_Init(j, PME_JOINT_PRISMATIC, b, 2, 0, 1, V(0, 0, 0), frame) == PME_OK);
    b[1].velocity = V(0, 1, 0);
    CHECK(step(b, 2, j, 1, V(0, 0, 0), 1));
    momentum = V(0, 0, 0);
    for (i = 0; i < 2; ++i)
        momentum =
            YMPME_VecAdd(momentum, YMPME_VecAdd(b[i].angularVelocity,
                                                YMPME_VecCross(b[i].position, b[i].velocity)));
    CHECK(NEAR(momentum.z, 1, .0001f));
    CHECK(norm(YMPME_VecAdd(j->feedback.reactionA.force, j->feedback.reactionB.force)) < .0001f);
    /* Chain with a fixed and revolute connection; all bodies solved in one array. */
    for (i = 0; i < 4; ++i)
    {
        CHECK(YMPME_Rigid_Init(&b[i], 1, V(.1f, .1f, .1f), V(0, -i - .5f, 0), Q) == PME_OK);
        CHECK(YMPME_Joint_Init(&j[i], i == 2 ? PME_JOINT_FIXED : PME_JOINT_REVOLUTE, b, 4,
                               i ? (size_t)i - 1 : PME_JOINT_WORLD, (size_t)i, V(0, -i, 0),
                               Q) == PME_OK);
    }
    CHECK(YMPME_Rigid_AddForceAt(&b[3], V(50, 0, 0), b[3].position) == PME_OK);
    CHECK(step(b, 4, j, 4, V(0, -9.81f, 0), 1200));
    for (i = 0; i < 4; ++i)
    {
        CHECK(j[i].feedback.anchorError < .015f);
        CHECK(j[i].feedback.angularError < .015f);
    }
    /* Validation must occur before any body is advanced. */
    saved = b[0];
    js = j[0];
    j[3].bodyB = 4;
    CHECK(YMPME_Joint_Step(b, 4, j, 4, V(0, 0, 0), .01f, 32, 6) == PME_INVALID_ARGUMENT);
    CHECK(!memcmp(b, &saved, sizeof(saved)) && !memcmp(j, &js, sizeof(js)));
    j[3].bodyB = 3;
    c = b[3].position.y;
    j[3].enabled = 0;
    CHECK(step(b, 4, j, 4, V(0, -9.81f, 0), 240));
    CHECK(b[3].position.y < c - 1);
    CHECK(norm(j[3].feedback.reactionB.force) == 0);
    puts("Joints: fixed support, 3D momentum, continuous angle, motors/limits, slider, chain, "
         "release, validation passed");
    return 0;
}
