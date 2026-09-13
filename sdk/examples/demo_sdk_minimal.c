/* Only the core library is linked; optional modules must remain disabled. */
#include "YMPME.h"
#include <stdio.h>
#if !defined(PME_SDK_UNIFIED) && (PME_ENABLE_DYNAMICS || PME_ENABLE_RIGID || PME_ENABLE_FLIGHT || PME_MODEL_SKIN || PME_SENSOR_ENCODER)
#error "Core-only consumers must not acquire optional features"
#endif
int main(void)
{
    pme_world world;
    pme_body storage[1];
    pme_body_desc desc = YMPME_Body_DefaultDesc();
    if (YMPME_World_Init(&world, storage, 1, NULL) != PME_OK) return 1;
    desc.position.y = 10;
    PME_Body body = YMPME_World_AddBody(&world, &desc);
    if (!body) return 1;
    const float dt = 1.f / 120;
    for (int i = 0; i < 120; ++i) {
        if (YMPME_World_Step(&world, dt) != PME_OK) return 1;
    }
    float y = body->state.position.y, vy = body->state.velocity.y;
    printf("Free fall: y=%.6f m, vy=%.6f m/s\n", y, vy);
    YMPME_World_Destroy(&world);
    if (!isfinite(y) || fabsf(y - 5.095f) > .1f || fabsf(vy + 9.81f) > .02f) return 1;
    puts("PASS: free fall with external caller-owned storage, no GRE");
    return 0;
}
