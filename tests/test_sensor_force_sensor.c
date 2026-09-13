#include "SENSOR/FORCE/YMPME_ForceSensor.h"
#include "CORE/YMPME_World.h"
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
int main(void)
{
    pme_body storage[3];
    pme_world w;
    CHECK(YMPME_World_Init(&w, storage, 3, NULL) == PME_OK);
    pme_body_desc d = YMPME_Body_DefaultDesc();
    d.type = PME_STATIC;
    d.shape = PME_BOX;
    d.halfExtents = YMPME_Vec3(4, .5f, 4);
    d.position = YMPME_Vec3(0, -.5f, 0);
    CHECK(YMPME_World_AddBody(&w, &d));
    d = YMPME_Body_DefaultDesc();
    d.mass = 2;
    d.velocity.x = 3;
    d.position = YMPME_Vec3(0, .5f, 0);
    d.restitution = 0;
    CHECK(YMPME_World_AddBody(&w, &d));
    pme_body plainStorage[3];
    memcpy(plainStorage, storage, sizeof(storage));
    pme_world plain = w;
    plain.bodies = plainStorage;
    float frictionImpulse = 0;
    pme_feedback_state state[3];
    pme_contact_feedback events[32];
    pme_world_feedback feedback = {state, 3, events, 32, 0, 0, 0, 0};
    pme_force_window window;
    CHECK(YMPME_ForceWindow_Begin(&window, 1, storage[1].state.position) == PME_OK);
    for (int i = 0; i < 120; ++i)
    {
        feedback.time = (i + 1) / 120.0;
        CHECK(YMPME_World_StepWithFeedback(&w, 1.0f / 120, &feedback) == PME_OK);
        CHECK(YMPME_ForceWindow_Add(&window, &feedback) == PME_OK);
        CHECK(YMPME_World_Step(&plain, 1.0f / 120) == PME_OK);
        CHECK(!memcmp(&storage[1], &plainStorage[1], sizeof(storage[1])));
        for (size_t k = 0; k < feedback.count; ++k)
            frictionImpulse += fabsf(feedback.contacts[k].tangentImpulse.x);
    }
    CHECK(frictionImpulse > 1);
    pme_force_sample f, g;
    CHECK(YMPME_ForceWindow_Read(&window, &f) == PME_OK);
    CHECK(fabsf(f.wrench.force.y - 19.62f) < .01f);
    CHECK(YMPME_ForceWindow_Add(&window, &feedback) == PME_INVALID_ARGUMENT);
    CHECK(YMPME_ForceSensor_Contacts(&feedback, 1, state[1].position, &f) == PME_OK);
    CHECK(fabsf(f.wrench.force.y - 19.62f) < .01f);
    CHECK(YMPME_ForceSensor_Contacts(&feedback, 0, state[0].position, &g) == PME_OK);
    CHECK(fabsf(f.wrench.force.y + g.wrench.force.y) < 1e-5f);
    CHECK(f.approximateTorque);
    CHECK(!(state[1].valid & PME_FEEDBACK_ANGULAR_VELOCITY));
    CHECK(state[2].valid == 0);
    CHECK(fabsf(state[1].acceleration.y) < .01f);
    pme_sensor_mount mount = YMPME_Sensor_IdentityMount();
    mount.offset = YMPME_Vec3(1, 0, 0);
    pme_wrench input = {YMPME_Vec3(0, 10, 0), {0, 0, 0}};
    pme_sensor_stamp stamp = {state[1].time, state[1].time, 1, 1};
    CHECK(YMPME_ForceSensor_Measure(&input, &state[1], &mount, &stamp, &f) == PME_OK);
    CHECK(fabsf(f.wrench.torque.z + 10) < 1e-5f);
    feedback.contactCapacity = 0;
    feedback.time += 1.0 / 120;
    CHECK(YMPME_World_StepWithFeedback(&w, 1.0f / 120, &feedback) == PME_OK);
    CHECK(feedback.dropped > 0);
    CHECK(YMPME_ForceSensor_Contacts(&feedback, 1, state[1].position, &f) == PME_INVALID_ARGUMENT);
    pme_body before = storage[1];
    feedback.stateCapacity = 0;
    CHECK(YMPME_World_StepWithFeedback(&w, 1.0f / 120, &feedback) == PME_INVALID_ARGUMENT);
    CHECK(!memcmp(&before, &storage[1], sizeof(before)));
    puts("PASS contact force, action/reaction, validity, lever arm torque and overflow rejection");
    return 0;
}
