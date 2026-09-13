#include "SENSOR/TACTILE/YMPME_TactileWindow.h"
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

    pme_taxel cells[2] = {0}, integrals[2], averages[2], savedCells[2];
    pme_tactile_window w, saved;
    pme_tactile_frame f = {0}, out = {0};
    f.cells = cells;
    f.capacity = 2;
    f.width = 2;
    f.height = 1;
    f.rotation = YMPME_Rotation_Identity();
    out.cells = averages;
    out.capacity = 2;
    CHECK(YMPME_TactileWindow_Begin(&w, integrals, 2) == PME_OK);
    CHECK(YMPME_TactileWindow_Read(&w, &out) == PME_INVALID_ARGUMENT);
    f.stamp = (pme_sensor_stamp){.25, .9, 1, 1};
    f.accepted = 1;
    f.approximate = 1;
    cells[0] = (pme_taxel){{0, 0, -4}, 8, 1};
    CHECK(YMPME_TactileWindow_Add(&w, &f, .25f) == PME_OK);
    saved = w;
    memcpy(savedCells, integrals, sizeof integrals);
    CHECK(YMPME_TactileWindow_Add(&w, &f, .25f) == PME_INVALID_ARGUMENT);
    CHECK(!memcmp(&w, &saved, sizeof w) && !memcmp(integrals, savedCells, sizeof integrals));
    f.stamp.time = .75;
    CHECK(YMPME_TactileWindow_Add(&w, &f, .25f) == PME_INVALID_ARGUMENT);
    f.stamp.time = .5;
    f.origin.x = 2;
    cells[0].force.z = -8;
    cells[0].pressure = 16;
    cells[1].pressure = NAN;
    CHECK(YMPME_TactileWindow_Add(&w, &f, .25f) == PME_INVALID_ARGUMENT);
    CHECK(!memcmp(&w, &saved, sizeof w) && !memcmp(integrals, savedCells, sizeof integrals));
    cells[1].pressure = 0;
    f.stamp.time = f.stamp.availableTime = 1;
    CHECK(YMPME_TactileWindow_Add(&w, &f, .75f) == PME_OK);
    CHECK(YMPME_TactileWindow_Read(&w, &out) == PME_OK);
    CHECK(NEAR(out.cells[0].force.z, -7) && NEAR(out.cells[0].pressure, 14) &&
          NEAR(out.totalForce.z, -7));
    CHECK(out.origin.x == 2 && out.stamp.time == 1 && out.stamp.availableTime == 1 &&
          out.accepted == 2 && out.approximate && out.cells[0].contributions == 2);
    CHECK(YMPME_TactileWindow_Read(&w, &out) == PME_OK && NEAR(out.totalForce.z, -7));
    saved = w;
    memcpy(savedCells, integrals, sizeof integrals);
    f.stamp.time = f.stamp.availableTime = 3;
    cells[1].pressure = FLT_MAX;
    CHECK(YMPME_TactileWindow_Add(&w, &f, 2) == PME_INVALID_ARGUMENT);
    CHECK(!memcmp(&w, &saved, sizeof w) && !memcmp(integrals, savedCells, sizeof integrals));
    puts("Tactile window: weighted impulse conservation, moving skin, gaps and atomic overflow "
         "passed");

    return 0;
}
