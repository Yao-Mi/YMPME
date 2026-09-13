#include "MODEL/AIRCRAFT/YMPME_Aircraft.h"
#include <stdio.h>
#include <string.h>
#define CHECK(v)                                                                                   \
    do                                                                                             \
    {                                                                                              \
        if (!(v))                                                                                  \
        {                                                                                          \
            fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #v);                                   \
            return 1;                                                                              \
        }                                                                                          \
    } while (0)
static int run(pme_aircraft *a, pme_aircraft_command c, pme_vec3 wind, int steps)
{
    for (int i = 0; i < steps; ++i)
        if (YMPME_Aircraft_Step(a, &c, wind, 1.0f / 240) != PME_OK)
            return 0;
    return 1;
}
int main(void)
{
    pme_aircraft d, e;
    pme_vec3 zero = {0, 0, 0};
    pme_aircraft_command c = {0, 0, 0, 0, 1};
    CHECK(YMPME_Aircraft_Init(&d, PME_AIRCRAFT_DRONE, YMPME_Vec3(0, 10, 0)) == PME_OK);
    CHECK(run(&d, c, zero, 2400));
    CHECK(fabsf(d.position.y - 10) < .05f);
    CHECK(fabsf(d.position.x) + fabsf(d.position.z) < .001f);
    c.forward = .6f;
    CHECK(run(&d, c, zero, 720));
    CHECK(d.position.z > 4);
    CHECK(fabsf(d.position.y - 10) < .7f);
    c.forward = 0;
    CHECK(run(&d, c, YMPME_Vec3(3, 0, 0), 1200));
    CHECK(fabsf(d.velocity.z) < .15f);
    CHECK(fabsf(d.position.y - 10) < 1);
    c.turn = .5f;
    CHECK(run(&d, c, zero, 480));
    CHECK(d.angularVelocity.y > .4f);
    c.turn = 0;
    c.climb = .5f;
    float h = d.position.y;
    CHECK(run(&d, c, zero, 480));
    CHECK(d.position.y > h + 1);
    c.climb = 0;
    c.power = 0;
    CHECK(run(&d, c, zero, 2400));
    CHECK(d.position.y == .3f);
    pme_aircraft saved = d;
    c.power = 2;
    CHECK(YMPME_Aircraft_Step(&d, &c, zero, 1.0f / 240) == PME_INVALID_ARGUMENT);
    CHECK(!memcmp(&saved, &d, sizeof(d)));
    c.power = 1;
    CHECK(YMPME_Aircraft_Step(&d, &c, zero, .02f) == PME_INVALID_ARGUMENT);
    CHECK(!memcmp(&saved, &d, sizeof(d)));
    CHECK(YMPME_Aircraft_Init(&e, PME_AIRCRAFT_EAGLE, YMPME_Vec3(0, 80, 0)) == PME_OK);
    CHECK(run(&e, c, zero, 2400));
    printf("eagle powered: h=%.2f speed=%.2f lift=%.2f\n", e.position.y, e.airspeed, e.lift);
    CHECK(e.position.y > 70);
    CHECK(e.position.z > 60);
    CHECK(e.airspeed > 6 && e.airspeed < 25);
    c.power = 0;
    float energy0 = e.mass * (9.81f * e.position.y + .5f * YMPME_VecDot(e.velocity, e.velocity));
    h = e.position.y;
    CHECK(run(&e, c, zero, 1200));
    float energy1 = e.mass * (9.81f * e.position.y + .5f * YMPME_VecDot(e.velocity, e.velocity));
    printf("glide: h=%.2f speed=%.2f energy %.2f -> %.2f\n", e.position.y, e.airspeed, energy0,
           energy1);
    CHECK(energy1 < energy0);
    CHECK(e.position.y < h);
    CHECK(fabsf(e.wingAngle) < .001f);
    CHECK(e.thrust == 0);
    c.power = 1;
    c.turn = .6f;
    CHECK(run(&e, c, zero, 2400));
    CHECK(e.position.x > 5);
    CHECK(e.position.y > .3f);
    float norm = 0;
    for (int i = 0; i < 4; ++i)
        norm += e.q[i] * e.q[i];
    CHECK(fabsf(norm - 1) < 1e-5f);
    CHECK(!memcmp(&saved, &d, sizeof(d)));
    CHECK(YMPME_Aircraft_Init(&e, PME_AIRCRAFT_EAGLE, YMPME_Vec3(0, 12, 0)) == PME_OK);
    for (int i = 0; i < 240 * 120; ++i)
    {
        c = (pme_aircraft_command){
            0, 0, YMPME_Clamp((12 - e.position.y) * .12f - e.velocity.y * .14f, -.5f, .5f), .85f,
            (i / 240) % 16 < 12 ? 1 : 0};
        CHECK(YMPME_Aircraft_Step(&e, &c, YMPME_Vec3(2, 0, .5f), 1.0f / 240) == PME_OK);
        if (!(e.position.y > 3 && e.position.y < 22))
            fprintf(stderr, "at %.1fs: height %.2f speed %.2f vy %.2f power %.1f\n", i / 240.0,
                    e.position.y, e.airspeed, e.velocity.y, c.power);
        CHECK(e.position.y > 3 && e.position.y < 22);
    }
    printf("120s banked flap/glide with wind: height %.2f speed %.2f\n", e.position.y, e.airspeed);
    puts("PASS aircraft hover, steering, wind, climb, power loss, glide energy "
         "and independence");
    return 0;
}
