#include "SENSOR/FORCE/YMPME_ForceDevice.h"
#include <stdio.h>
#include <string.h>
#include <float.h>
#define CHECK(x)                                                                                   \
    do                                                                                             \
    {                                                                                              \
        if (!(x))                                                                                  \
        {                                                                                          \
            fprintf(stderr, "FAIL %d: %s\n", __LINE__, #x);                                        \
            return 1;                                                                              \
        }                                                                                          \
    } while (0)
#define NEAR(a, b) (fabs((double)(a) - (b)) < 1e-5)
int main(void)
{

    pme_force_device_desc desc = YMPME_ForceDevice_DefaultDesc();
    pme_force_device d, copy, saved;
    pme_force_reading out = {0}, other, old;
    pme_force_sample in = {0};
    pme_feedback_state s = {0};
    s.rotation = YMPME_Rotation_Identity();
    s.valid = PME_FEEDBACK_POSE;
    in.stamp = (pme_sensor_stamp){0, 0, 0, 1};
    in.wrench.force = YMPME_Vec3(0, -10, 0);
    in.approximateTorque = 1;
    desc.mount.offset = YMPME_Vec3(1, 0, 0);
    desc.bias.force.y = 2;
    CHECK(YMPME_ForceDevice_Init(&d, &desc, 42) == PME_OK);
    CHECK(YMPME_ForceDevice_Measure(&d, &in, &s, &out) == PME_OK &&
          out.sample.wrench.force.y == -8 && out.sample.wrench.torque.z == 10 &&
          out.sample.approximateTorque);
    YMPME_ForceDevice_TareNext(&d);
    saved = d;
    old = out;
    CHECK(YMPME_ForceDevice_Measure(&d, &in, &s, &out) == PME_INVALID_ARGUMENT);
    CHECK(!memcmp(&d, &saved, sizeof d) && !memcmp(&out, &old, sizeof out) && d.tarePending);
    s.time = in.stamp.time = in.stamp.availableTime = 1;
    CHECK(YMPME_ForceDevice_Measure(&d, &in, &s, &out) == PME_OK && !d.tarePending &&
          out.sample.wrench.force.y == 0 && out.sample.wrench.torque.z == 0);
    s.time = in.stamp.time = in.stamp.availableTime = 2;
    in.wrench.force.y = -15;
    CHECK(YMPME_ForceDevice_Measure(&d, &in, &s, &out) == PME_OK &&
          out.sample.wrench.force.y == -5 && out.sample.wrench.torque.z == 5);
    desc.force.minimum = -6;
    desc.force.maximum = 6;
    desc.force.resolution = .5f;
    CHECK(YMPME_ForceDevice_Init(&d, &desc, 42) == PME_OK);
    CHECK(YMPME_ForceDevice_Measure(&d, &in, &s, &out) == PME_OK &&
          out.sample.wrench.force.y == -6 && out.saturated == 2);
    desc.forceNoise = .1f;
    desc.torqueNoise = .01f;
    CHECK(YMPME_ForceDevice_Init(&d, &desc, 42) == PME_OK);
    copy = d;
    CHECK(YMPME_ForceDevice_Measure(&d, &in, &s, &out) == PME_OK);
    CHECK(YMPME_ForceDevice_Measure(&copy, &in, &s, &other) == PME_OK && d.seed == copy.seed &&
          d.seed != 42 && !memcmp(&out.sample.wrench, &other.sample.wrench, sizeof(pme_wrench)));
    s.time = in.stamp.time = in.stamp.availableTime = 3;
    d.desc.torque.maximum = d.desc.torque.minimum;
    YMPME_ForceDevice_TareNext(&d);
    saved = d;
    old = out;
    CHECK(YMPME_ForceDevice_Measure(&d, &in, &s, &out) == PME_INVALID_ARGUMENT);
    CHECK(!memcmp(&d, &saved, sizeof d) && !memcmp(&out, &old, sizeof out));
    puts("Force device: lever arm, approximate flag, tare, limits, noise, atomic failure passed");

    return 0;
}
