/* uint16 angle codes with int16 wrapped increments, as used by a controller. */
#include "SENSOR/ENCODER/YMPME_Encoder.h"
#include <stdio.h>
#include <stdint.h>
int main(void)
{
    pme_encoder encoder;
    pme_encoder_sample sample;
    if (YMPME_Encoder_Init(&encoder, 65536) != PME_OK) return 1;
    const double radiansPerCode = 6.28318530717958647692 / 65536;
    pme_sensor_stamp stamp = {0, 0, 1, 1};
    if (YMPME_Encoder_Measure(&encoder, 65530 * radiansPerCode, &stamp, &sample) != PME_OK) return 1;
    uint16_t last = sample.angle;
    stamp.time = .01;
    stamp.availableTime = .01;
    if (YMPME_Encoder_Measure(&encoder, 5 * radiansPerCode, &stamp, &sample) != PME_OK) return 1;
    uint16_t now = sample.angle;
    int16_t delta = (int16_t)(now - last);
    int32_t accumulated = 65530;
    accumulated += delta;
    printf("last=%u now=%u delta=%d accumulated=%ld\n", (unsigned)last,
        (unsigned)now, (int)delta, (long)accumulated);
    if (last != 65530 || now != 5 || delta != 11 || accumulated != 65541) return 1;
    puts("PASS: uint16 encoder wrap (inter-sample motion must be less than half a turn)");
    return 0;
}
