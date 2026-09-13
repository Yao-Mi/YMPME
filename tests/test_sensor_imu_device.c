#include "SENSOR/IMU/YMPME_IMUDevice.h"
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

    pme_imu_desc d = {0};
    d.mount = YMPME_Sensor_IdentityMount();
    d.gravity = YMPME_Vec3(0, -9.81f, 0);
    pme_signal_desc a = YMPME_Signal_DefaultDesc(), g = a;
    pme_imu_device dev, copy, saved;
    pme_imu_reading out = {0}, other, old;
    pme_feedback_state s = {0};
    s.rotation = YMPME_Rotation_Identity();
    s.valid = 31;
    pme_sensor_stamp t = {0, 0, 0, 1};
    a.minimum = -8;
    a.maximum = 8;
    a.resolution = .5f;
    CHECK(YMPME_IMUDevice_Init(&dev, &d, &a, &g, 42) == PME_OK && dev.seed == 42);
    CHECK(YMPME_IMUDevice_Measure(&dev, &s, &t, &out) == PME_OK &&
          out.sample.specificForce.y == 8 && out.saturated == 2);
    d.accelNoise = .1f;
    d.gyroNoise = .01f;
    a = YMPME_Signal_DefaultDesc();
    CHECK(YMPME_IMUDevice_Init(&dev, &d, &a, &g, 42) == PME_OK);
    copy = dev;
    CHECK(YMPME_IMUDevice_Measure(&dev, &s, &t, &out) == PME_OK);
    CHECK(YMPME_IMUDevice_Measure(&copy, &s, &t, &other) == PME_OK);
    CHECK(dev.seed == copy.seed && dev.seed != 42 &&
          !memcmp(&out.sample.specificForce, &other.sample.specificForce, sizeof(pme_vec3)));
    /* A later channel failure must not consume even the accelerometer's noise/filter update. */
    dev.gyroscope.minimum = dev.gyroscope.maximum;
    saved = dev;
    old = out;
    s.time = t.time = t.availableTime = 1;
    CHECK(YMPME_IMUDevice_Measure(&dev, &s, &t, &out) == PME_INVALID_ARGUMENT);
    CHECK(!memcmp(&dev, &saved, sizeof dev) && !memcmp(&out, &old, sizeof out));
    d.accelNoise = d.gyroNoise = 0;
    a.cutoffHz = (float)(log(2) / 6.283185307179586);
    CHECK(YMPME_IMUDevice_Init(&dev, &d, &a, &g, 42) == PME_OK);
    CHECK(YMPME_IMUDevice_Measure(&dev, &s, &t, &out) == PME_OK);
    s.time = t.time = t.availableTime = 2;
    s.acceleration.x = 10;
    CHECK(YMPME_IMUDevice_Measure(&dev, &s, &t, &out) == PME_OK &&
          NEAR(out.sample.specificForce.x, 5));
    puts("IMU device: limits, ADC, deterministic noise, filtering and atomic state passed");

    return 0;
}
