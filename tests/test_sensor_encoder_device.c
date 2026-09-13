#include "SENSOR/ENCODER/YMPME_EncoderDevice.h"
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

    const double tau = 6.2831853071795864769;
    pme_encoder_device d, copy, saved;
    pme_encoder_sample out = {0}, other, old;
    pme_sensor_stamp t = {0, 0, 0, 1};
    CHECK(YMPME_EncoderDevice_Init(&d, 0, tau / 4, -1, 0, 7) == PME_OK);
    CHECK(YMPME_EncoderDevice_Measure(&d, 0, &t, &out) == PME_OK && out.angle == 16384 &&
          !out.velocityValid);
    CHECK(YMPME_EncoderDevice_Init(&d, 0, 0, 1, 0, 7) == PME_OK);
    CHECK(YMPME_EncoderDevice_Measure(&d, tau * 65530 / 65536, &t, &out) == PME_OK);
    uint16_t last = out.angle;
    t.time = t.availableTime = 1;
    CHECK(YMPME_EncoderDevice_Measure(&d, tau * 5 / 65536, &t, &out) == PME_OK);
    uint16_t now = out.angle;
    int16_t delta = (int16_t)(now - last);
    CHECK(delta == 11 && out.velocityValid && NEAR(out.velocity, 11 * tau / 65536));
    CHECK(YMPME_EncoderDevice_Zero(&d, tau / 3) == PME_OK);
    t.time = t.availableTime = 2;
    CHECK(YMPME_EncoderDevice_Measure(&d, tau / 3, &t, &out) == PME_OK && out.angle == 0 &&
          !out.velocityValid);
    CHECK(YMPME_EncoderDevice_Init(&d, 0, 0, 1, .01f, 7) == PME_OK);
    copy = d;
    CHECK(YMPME_EncoderDevice_Measure(&d, 1, &t, &out) == PME_OK);
    CHECK(YMPME_EncoderDevice_Measure(&copy, 1, &t, &other) == PME_OK && out.angle == other.angle &&
          d.seed == copy.seed && d.seed != 7);
    saved = d;
    old = out;
    CHECK(YMPME_EncoderDevice_Measure(&d, 2, &t, &out) == PME_INVALID_ARGUMENT);
    CHECK(!memcmp(&d, &saved, sizeof d) && !memcmp(&out, &old, sizeof out));
    puts("Encoder device: reverse mount, zero, uint16/int16 wrap, deterministic noise passed");

    return 0;
}
