#include "SENSOR/ENCODER/YMPME_Encoder.h"
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
#define TAU 6.28318530717958647692
static int measure(pme_encoder *e, double angle, double time, pme_encoder_sample *out)
{
    pme_sensor_stamp stamp = {time, time, 1, 1};
    return YMPME_Encoder_Measure(e, angle, &stamp, out) == PME_OK;
}
int main(void)
{
    pme_encoder e;
    pme_encoder_sample out;
    CHECK(sizeof(out.angle) == sizeof(uint16_t));
    CHECK(YMPME_Encoder_Init(&e, 0) == PME_OK);
    CHECK(measure(&e, 0, 0, &out));
    CHECK(out.angle == 0 && out.stamp.valid && !out.velocityValid && out.velocity == 0);
    CHECK(measure(&e, TAU * .25, 1, &out));
    CHECK(out.angle == 16384 && out.velocityValid);
    CHECK(measure(&e, TAU * .5, 2, &out));
    CHECK(out.angle == 32768);
    CHECK(measure(&e, TAU * .75, 3, &out));
    CHECK(out.angle == 49152);
    CHECK(measure(&e, TAU, 4, &out));
    CHECK(out.angle == 0);
    CHECK(measure(&e, -TAU * .25, 5, &out));
    CHECK(out.angle == 49152);
    CHECK(measure(&e, TAU * 4.25, 6, &out));
    CHECK(out.angle == 16384);
    /* 穷举16位码值往返，包含最大值；每步仅一个刻度，不发生速度混叠。 */
    CHECK(YMPME_Encoder_Init(&e, 65536) == PME_OK);
    for (unsigned i = 0; i < 65536; ++i)
    {
        CHECK(measure(&e, i * (TAU / 65536), i * .01, &out));
        CHECK(out.angle == i);
    }
    CHECK(measure(&e, TAU, 655.36, &out));
    CHECK(out.angle == 0);
    CHECK(fabsf(out.velocity - (float)(TAU / 65536 / .01)) < 1e-6f);
    CHECK(measure(&e, TAU - TAU / 65536, 655.37, &out));
    CHECK(out.angle == 65535 && out.velocity < 0);
    CHECK(measure(&e, TAU / 65536 * .49, 656, &out));
    CHECK(out.angle == 0);
    CHECK(measure(&e, TAU / 65536 * .51, 657, &out));
    CHECK(out.angle == 1);
    pme_encoder saved = e;
    pme_encoder_sample previous = out;
    CHECK(!measure(&e, NAN, 658, &out));
    CHECK(!memcmp(&e, &saved, sizeof(e)) && !memcmp(&out, &previous, sizeof(out)));
    CHECK(!measure(&e, 0, 657, &out));
    CHECK(!memcmp(&e, &saved, sizeof(e)));
    CHECK(YMPME_Encoder_Init(&e, 65537) == PME_INVALID_ARGUMENT);
    CHECK(!memcmp(&e, &saved, sizeof(e)));
    CHECK(YMPME_Encoder_Init(&e, 4) == PME_OK);
    CHECK(measure(&e, .1, 0, &out));
    CHECK(out.angle == 0);
    CHECK(measure(&e, 1.6, 1, &out));
    CHECK(out.angle == 16384);
    CHECK(sizeof(e.lastAngle) == sizeof(uint16_t));
    CHECK(YMPME_Encoder_Init(&e, 65536) == PME_OK);
    CHECK(measure(&e, 65530 * (TAU / 65536), 0, &out));
    CHECK(measure(&e, 5 * (TAU / 65536), .01, &out));
    CHECK(out.angle == 5);
    CHECK(fabsf(out.velocity - (float)(11 * TAU / 65536 / .01)) < 1e-6f);
    CHECK(measure(&e, 65530 * (TAU / 65536), .02, &out));
    CHECK(out.angle == 65530);
    CHECK(fabsf(out.velocity + (float)(11 * TAU / 65536 / .01)) < 1e-6f);
    puts("PASS uint16 angle: all 65536 codes, degree mapping, negative/multiturn, wrap direction, "
         "quantization and validity");
    return 0;
}
