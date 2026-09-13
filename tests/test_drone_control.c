#include "control_rig.h"
#include <stdio.h>
#include <string.h>
#define CHECK(x)                                                                                   \
    do                                                                                             \
    {                                                                                              \
        if (!(x))                                                                                  \
        {                                                                                          \
            fprintf(stderr, "FAIL %d: %s\n", __LINE__, #x);                                        \
            return 1;                                                                              \
        }                                                                                          \
    } while (0)
int main(void)
{
    pme_aircraft a;
    CHECK(YMPME_Aircraft_Init(&a, PME_AIRCRAFT_DRONE, YMPME_Vec3(0, 6, 0)) == PME_OK);
    pme_feedback_state feedback;
    CHECK(YMPME_Aircraft_GetFeedback(&a, &feedback) == PME_OK);
    CHECK(!(feedback.valid & PME_FEEDBACK_ACCELERATION));
    float motor[4] = {.8f, .4f, .8f, .4f};
    pme_vec3 zero = {0, 0, 0};
    for (int i = 0; i < 100; ++i)
        CHECK(YMPME_Aircraft_StepMotors(&a, motor, zero, 1.0f / 240) == PME_OK);
    CHECK(fabsf(a.angularVelocity.z) > .8f); /* 非对称电机真的产生转动，内置稳定器没有接管。 */
    pme_aircraft saved = a;
    motor[1] = 2;
    CHECK(YMPME_Aircraft_StepMotors(&a, motor, zero, 1.0f / 240) == PME_INVALID_ARGUMENT);
    CHECK(!memcmp(&a, &saved, sizeof(a)));
    CHECK(YMPME_Aircraft_Init(&a, PME_AIRCRAFT_DRONE, YMPME_Vec3(0, 6, 0)) == PME_OK);
    drone_control_rig rig;
    DroneControlRig_Init(&rig);
    pme_aircraft_command c = {0, 0, 0, 0, 1};
    for (int i = 0; i < 240 * 30; ++i)
    {
        c.climb = YMPME_Clamp((6 - a.position.y) * .5f, -1, 1);
        if (i == 240 * 3)
        {
            a.velocity.x += 2;
            a.angularVelocity.z += 1;
        }
        CHECK(DroneControlRig_Step(&rig, &a, &c, YMPME_Vec3(2, 0, .5f), 1.0f / 240));
        CHECK(a.position.y > 5 && a.position.y < 7);
    }
    CHECK(fabsf(a.velocity.x) < .3f);
    CHECK(fabsf(a.angularVelocity.z) < .03f);
    CHECK(rig.updates > 3500 && rig.updates < 3650);
    CHECK(rig.enc[0].stamp.valid && rig.enc[0].velocityValid);
    CHECK(fabsf(fabsf(rig.enc[0].velocity) - a.rotorSpeed[0]) < 3);
    CHECK(fabsf(rig.imu.specificForce.y - 9.81f) < .5f);
    CHECK(YMPME_Aircraft_GetFeedback(&a, &feedback) == PME_OK);
    CHECK(feedback.valid == 31);
    pme_imu_sample stale = rig.imu;
    stale.stamp.time -= 1;
    CHECK(!DroneController_Update(&rig.controller, &feedback, &stale, &c, 1.0f / 120, motor));
    c.climb = .6f;
    for (int i = 0; i < 480; ++i)
        CHECK(DroneControlRig_Step(&rig, &a, &c, zero, 1.0f / 240));
    CHECK(a.position.y > 7.5f);
    c.power = 0;
    for (int i = 0; i < 1200; ++i)
        CHECK(DroneControlRig_Step(&rig, &a, &c, zero, 1.0f / 240));
    CHECK(a.grounded);
    CHECK(fabsf(rig.imu.specificForce.y - 9.81f) < .1f); /* 落地反力包含在速度差分中。 */
    puts("PASS raw motors, no hidden stabilizer, 240Hz sensors/120Hz external control, disturbance "
         "recovery, encoder, stale sample, climb and ground IMU");
    return 0;
}
