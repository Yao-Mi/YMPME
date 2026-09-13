#include "SENSOR/YMPME_Signal.h"
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

    pme_signal_desc d = YMPME_Signal_DefaultDesc();
    pme_signal_state s = {0}, saved;
    float out = 99;
    int clipped = 99;
    d.cutoffHz = (float)(log(2) / 6.283185307179586);
    CHECK(YMPME_Signal_Process(&d, &s, 0, 0, &out, &clipped) == PME_OK && out == 0 && !clipped);
    CHECK(YMPME_Signal_Process(&d, &s, 10, 1, &out, &clipped) == PME_OK && NEAR(out, 5));
    CHECK(YMPME_Signal_Process(&d, &s, 10, 3, &out, &clipped) == PME_OK && NEAR(out, 8.75));
    saved = s;
    CHECK(YMPME_Signal_Process(&d, &s, 0, 3, &out, &clipped) == PME_INVALID_ARGUMENT);
    CHECK(!memcmp(&s, &saved, sizeof s) && NEAR(out, 8.75));
    d.cutoffHz = 0;
    d.gain = 2;
    d.offset = 1;
    d.minimum = -3;
    d.maximum = 3;
    d.resolution = .5f;
    CHECK(YMPME_Signal_Process(&d, &s, .63f, 4, &out, &clipped) == PME_OK && out == 2.5f &&
          !clipped);
    CHECK(YMPME_Signal_Process(&d, &s, -3, 5, &out, &clipped) == PME_OK && out == -3 && clipped);
    saved = s;
    CHECK(YMPME_Signal_Process(&d, &s, NAN, 6, &out, &clipped) == PME_INVALID_ARGUMENT);
    CHECK(!memcmp(&s, &saved, sizeof s) && out == -3 && clipped);
    d.minimum = d.maximum;
    CHECK(!YMPME_Signal_DescValid(&d));
    puts("Signal: actual-dt filter, gain, quantization, saturation, transactional errors passed");

    return 0;
}
