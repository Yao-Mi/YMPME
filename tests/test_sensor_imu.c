#include "SENSOR/IMU/YMPME_IMU.h"
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
    pme_feedback_state f = {0};
    f.rotation = YMPME_Rotation_Identity();
    f.valid = 31;
    f.time = 1;
    pme_imu_desc d = {0};
    d.mount = YMPME_Sensor_IdentityMount();
    d.gravity = YMPME_Vec3(0, -9.81f, 0);
    pme_sensor_stamp stamp = {1, 1, 1, 1};
    pme_imu_sample out;
    uint32_t rng = 42;
    CHECK(YMPME_IMU_Measure(&d, &f, &stamp, &rng, &out) == PME_OK);
    CHECK(fabsf(out.specificForce.y - 9.81f) < 1e-5f);
    f.acceleration = d.gravity;
    CHECK(YMPME_IMU_Measure(&d, &f, &stamp, &rng, &out) == PME_OK);
    CHECK(YMPME_VecDot(out.specificForce, out.specificForce) < 1e-9f);
    f.acceleration = YMPME_Vec3(0, 0, 0);
    f.angularVelocity = YMPME_Vec3(0, 2, 0);
    f.angularAcceleration = YMPME_Vec3(0, 3, 0);
    d.mount.offset = YMPME_Vec3(1, 0, 0);
    CHECK(YMPME_IMU_Measure(&d, &f, &stamp, &rng, &out) == PME_OK);
    CHECK(fabsf(out.specificForce.x + 4) < 1e-5f);
    CHECK(fabsf(out.specificForce.z + 3) < 1e-5f);
    d.mount.rotation.axis[0] = YMPME_Vec3(0, 1, 0);
    d.mount.rotation.axis[1] = YMPME_Vec3(-1, 0, 0);
    CHECK(YMPME_IMU_Measure(&d, &f, &stamp, &rng, &out) == PME_OK);
    CHECK(fabsf(out.angularVelocity.x - 2) < 1e-5f);
    pme_imu_sample saved = out;
    uint32_t old = rng;
    f.valid = PME_FEEDBACK_POSE;
    CHECK(YMPME_IMU_Measure(&d, &f, &stamp, &rng, &out) == PME_INVALID_ARGUMENT);
    CHECK(!memcmp(&out, &saved, sizeof(out)) && old == rng);
    f.valid = 31;
    d.accelNoise = .1f;
    uint32_t a = 7, b = 7;
    pme_imu_sample one, two;
    CHECK(YMPME_IMU_Measure(&d, &f, &stamp, &a, &one) == PME_OK);
    CHECK(YMPME_IMU_Measure(&d, &f, &stamp, &b, &two) == PME_OK);
    CHECK(!memcmp(&one.specificForce, &two.specificForce, sizeof(pme_vec3)));
    pme_sensor_clock clock;
    CHECK(YMPME_SensorClock_Init(&clock, .01, .005) == PME_OK);
    CHECK(YMPME_SensorClock_Stamp(&clock, 0, &stamp) == PME_OK);
    CHECK(stamp.availableTime == .005);
    CHECK(!YMPME_SensorClock_Due(&clock, .009));
    CHECK(YMPME_SensorClock_Stamp(&clock, .03, &stamp) == PME_OK);
    CHECK(clock.missed == 2 && stamp.sequence == 2);
    CHECK(!YMPME_SensorClock_Due(&clock, .02));
    puts("PASS IMU specific force, lever arm, mount, invalid data, deterministic noise and "
         "sampling clock");
    return 0;
}
