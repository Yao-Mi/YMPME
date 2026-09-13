#include "CORE/RIGID/YMPME_Rigid.h"
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
int main(void)
{
    pme_rigid_body b, saved;
    pme_quaternion q;
    pme_rotation r;
    pme_vec3 inertia, v;
    int i;
    CHECK(YMPME_Quaternion_AxisAngle(YMPME_Vec3(0, 0, 1), 1.570796327f, &q) == PME_OK);
    r = YMPME_Quaternion_Rotation(q);
    v = YMPME_Rotation_Apply(r, YMPME_Vec3(1, 0, 0));
    CHECK(NEAR(v.x, 0, 1e-6f) && NEAR(v.y, 1, 1e-6f));
    v = YMPME_Rotation_InverseApply(r, v);
    CHECK(NEAR(v.x, 1, 1e-6f));
    inertia = YMPME_Rigid_BoxInertia(12, YMPME_Vec3(1, 2, 3));
    CHECK(inertia.x == 13 && inertia.y == 10 && inertia.z == 5);
    CHECK(YMPME_Rigid_Init(&b, 2, YMPME_Vec3(1, 2, 3), YMPME_Vec3(0, 0, 0), q) == PME_OK);
    v = YMPME_Rigid_InverseInertia(&b, YMPME_Vec3(1, 0, 0));
    CHECK(NEAR(v.x, .5f, 1e-6f));
    CHECK(YMPME_Rigid_AddForceAt(&b, YMPME_Vec3(2, 0, 0), YMPME_Vec3(0, 1, 0)) == PME_OK);
    CHECK(YMPME_Rigid_Step(&b, YMPME_Vec3(0, 0, 0), .01f, .01) == PME_OK);
    CHECK(NEAR(b.velocity.x, .01f, 1e-6f) && NEAR(b.angularVelocity.z, -.02f / 3, 1e-6f));
    CHECK(NEAR(b.feedback.acceleration.x, 1, 1e-6f) && b.feedback.valid == 31);
    CHECK(b.load.force.x == 0 && b.load.torque.z == 0);
    saved = b;
    CHECK(YMPME_Rigid_Step(&b, YMPME_Vec3(0, 0, 0), 0, 1) == PME_INVALID_ARGUMENT);
    CHECK(!memcmp(&saved, &b, sizeof(b)));
    CHECK(YMPME_Rigid_Init(&b, 1, YMPME_Vec3(2, 2, 2), YMPME_Vec3(0, 0, 0),
                           YMPME_Quaternion_Identity()) == PME_OK);
    for (i = 0; i < 1000; ++i)
    {
        CHECK(YMPME_Rigid_AddTorque(&b, YMPME_Vec3(0, 0, 2)) == PME_OK);
        CHECK(YMPME_Rigid_Step(&b, YMPME_Vec3(0, 0, 0), .001f, (i + 1) * .001) == PME_OK);
    }
    CHECK(NEAR(b.angularVelocity.z, 1, 2e-5f));
    CHECK(NEAR(YMPME_Quaternion_Log(b.orientation).z, .5005f, 3e-5f));
    /* Arbitrarily small nonzero angular speeds remain finite at a settled servo. */
    b.angularVelocity = YMPME_Vec3(0, 0, 1e-7f);
    CHECK(YMPME_Rigid_Step(&b, YMPME_Vec3(0, 0, 0), .001f, 1.001) == PME_OK);
    CHECK(YMPME_Quaternion_IsValid(b.orientation));
    /* Rotation about a non-principal axis includes the gyroscopic term. */
    CHECK(YMPME_Rigid_Init(&b, 1, YMPME_Vec3(1, 2, 3), YMPME_Vec3(0, 0, 0),
                           YMPME_Quaternion_Identity()) == PME_OK);
    b.angularVelocity = YMPME_Vec3(1, 2, 3);
    CHECK(YMPME_Rigid_Step(&b, YMPME_Vec3(0, 0, 0), .001f, .001) == PME_OK);
    CHECK(NEAR(b.feedback.angularAcceleration.x, -6, .001f));
    CHECK(NEAR(b.feedback.angularAcceleration.y, 3, .001f));
    CHECK(YMPME_Rigid_Init(&b, 0, YMPME_Vec3(0, 0, 0), YMPME_Vec3(1, 2, 3), q) == PME_OK);
    CHECK(YMPME_Rigid_Step(&b, YMPME_Vec3(0, -9.81f, 0), .01f, .01) == PME_OK);
    CHECK(b.position.y == 2 && b.angularVelocity.z == 0);
    puts("Rigid: transforms, inertia, off-center loads, gyroscopic response, feedback, invalid "
         "inputs passed");
    return 0;
}
