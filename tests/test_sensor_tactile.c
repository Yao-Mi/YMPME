#include "SENSOR/TACTILE/YMPME_Tactile.h"
#include <stdio.h>
#include <string.h>
#define CHECK(x)                                                                                   \
    do                                                                                             \
    {                                                                                              \
        if (!(x))                                                                                  \
        {                                                                                          \
            fprintf(stderr, "%s:%d %s\n", __FILE__, __LINE__, #x);                                 \
            return 1;                                                                              \
        }                                                                                          \
    } while (0)
int main(void)
{
    pme_tactile_desc d = {YMPME_Sensor_IdentityMount(), 4, 2, .1f, .1f, .01f};
    pme_taxel cells[8];
    pme_tactile_frame out = {0}, saved;
    pme_feedback_state s = {0};
    pme_sensor_stamp t = {1, 1, 1, 1};
    pme_tactile_contact c[4] = {{{-.15f, -.05f, 0}, {2, 0, -5}, 0},
                                {{.15f, .05f, 0}, {-1, 1, -10}, 0},
                                {{.5f, 0, 0}, {0, 0, -9}, 0},
                                {{0, 0, .1f}, {0, 0, -9}, 0}};
    pme_contact_feedback contacts[2] = {0};
    pme_world_feedback feedback = {0};
    float sum = 0;
    size_t i;
    s.time = 1;
    s.valid = PME_FEEDBACK_POSE;
    s.rotation = YMPME_Rotation_Identity();
    out.cells = cells;
    out.capacity = 8;
    CHECK(YMPME_Tactile_Measure(&d, &s, &t, c, 4, &out) == PME_OK && out.accepted == 2 &&
          out.rejected == 2 && !out.approximate);
    CHECK(fabsf(out.totalForce.z + 15) < 1e-5f && out.totalForce.x == 1 && out.totalForce.y == 1);
    for (i = 0; i < 8; ++i)
        sum += cells[i].pressure * .01f;
    CHECK(fabsf(sum - 15) < 1e-4f);
    CHECK(fabsf(cells[0].pressure - 500) < .01f && fabsf(cells[7].pressure - 1000) < .01f);
    c[0].force.z = 1;
    CHECK(YMPME_Tactile_Measure(&d, &s, &t, c, 1, &out) == PME_OK && out.rejected == 1 &&
          out.totalForce.z == 0);
    /* A floor's outward normal is +Y; impulses on the other body must be inverted. */
    d.mount.rotation = (pme_rotation){{{1, 0, 0}, {0, 0, -1}, {0, 1, 0}}};
    contacts[0].bodyA = contacts[1].bodyA = 0;
    contacts[0].bodyB = contacts[1].bodyB = 1;
    contacts[0].point = contacts[1].point = YMPME_Vec3(0, 0, 0);
    contacts[0].normalImpulse = YMPME_Vec3(0, .1f, 0);
    contacts[1].normalImpulse = YMPME_Vec3(0, .2f, 0);
    contacts[1].tangentImpulse = YMPME_Vec3(.01f, 0, 0);
    contacts[0].approximatePoint = 1;
    feedback.contacts = contacts;
    feedback.count = feedback.contactCapacity = 2;
    feedback.stateCount = 2;
    feedback.time = 1;
    feedback.interval = .01f;
    CHECK(YMPME_Tactile_FromContacts(&d, &s, &t, &feedback, 0, &out) == PME_OK && out.approximate &&
          out.accepted == 2);
    CHECK(fabsf(out.totalForce.z + 30) < 1e-4f && fabsf(out.totalForce.x + 1) < 1e-5f);
    saved = out;
    feedback.dropped = 1;
    CHECK(YMPME_Tactile_FromContacts(&d, &s, &t, &feedback, 0, &out) == PME_INVALID_ARGUMENT &&
          !memcmp(&saved, &out, sizeof(out)));
    out.capacity = 1;
    CHECK(YMPME_Tactile_Measure(&d, &s, &t, c, 1, &out) == PME_CAPACITY_EXCEEDED);
    puts("Tactile: cell pressure, shear/force conservation, rejected loads, all impulse "
         "iterations, approximate flag and overflow passed");
    return 0;
}
