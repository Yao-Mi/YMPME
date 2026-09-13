#include "CORE/QUERY/YMPME_Query.h"
#include "SENSOR/LIDAR/YMPME_Lidar.h"
#include "SENSOR/DEPTH/YMPME_Depth.h"
#include "SENSOR/TACTILE/YMPME_Tactile.h"
#include <stdio.h>
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
    pme_world w;
    pme_body bodies[4];
    pme_body_desc b = YMPME_Body_DefaultDesc();
    pme_feedback_state states[4], sensor = {0};
    pme_contact_feedback contacts[100];
    pme_world_feedback feedback = {0};
    pme_sensor_stamp stamp;
    pme_world_query query;
    pme_ray_provider provider;
    pme_lidar_desc lidar = {YMPME_Sensor_IdentityMount(), .1f, 20, 0, 0};
    pme_vec3 beams[360], rays[9];
    pme_lidar_point points[360];
    pme_lidar_frame scan = {0};
    pme_depth_desc depth = {YMPME_Sensor_IdentityMount(), 3, 3, 3, 3, 1.5f, 1.5f, .1f, 20, 0, 0};
    float depths[9];
    pme_depth_frame image = {0};
    pme_tactile_desc tactile = {YMPME_Sensor_IdentityMount(), 8, 8, .5f, .5f, .02f};
    pme_taxel cells[64];
    pme_tactile_frame skin = {0};
    uint32_t seed = 1;
    int i;
    CHECK(YMPME_World_Init(&w, bodies, 4, NULL) == PME_OK);
    b.type = PME_STATIC;
    b.shape = PME_BOX;
    b.position = YMPME_Vec3(0, -.1f, 0);
    b.halfExtents = YMPME_Vec3(2, .1f, 2);
    CHECK(YMPME_World_AddBody(&w, &b));
    b.type = PME_DYNAMIC;
    b.shape = PME_SPHERE;
    b.position = YMPME_Vec3(0, 1, 0);
    b.radius = .3f;
    b.mass = 2;
    b.restitution = 0;
    CHECK(YMPME_World_AddBody(&w, &b));
    b.type = PME_STATIC;
    b.shape = PME_BOX;
    b.position = YMPME_Vec3(0, 1, 5);
    b.halfExtents = YMPME_Vec3(3, 2, .1f);
    CHECK(YMPME_World_AddBody(&w, &b));
    feedback.states = states;
    feedback.stateCapacity = 4;
    feedback.contacts = contacts;
    feedback.contactCapacity = 100;
    for (i = 0; i < 600; ++i)
    {
        feedback.time = (i + 1) / 240.;
        CHECK(YMPME_World_StepWithFeedback(&w, 1.f / 240, &feedback) == PME_OK);
    }
    stamp = (pme_sensor_stamp){feedback.time, feedback.time, 1, 1};
    tactile.mount.rotation = (pme_rotation){{{1, 0, 0}, {0, 0, -1}, {0, 1, 0}}};
    tactile.mount.offset.y = .1f;
    skin.cells = cells;
    skin.capacity = 64;
    CHECK(YMPME_Tactile_FromContacts(&tactile, &states[0], &stamp, &feedback, 0, &skin) == PME_OK &&
          skin.approximate);
    CHECK(fabsf(skin.totalForce.z + 19.62f) < .05f);
    query = (pme_world_query){&w, YMPME_Query_DefaultFilter()};
    provider = (pme_ray_provider){YMPME_Query_WorldRaycast, &query};
    sensor.position = YMPME_Vec3(0, 1, 0);
    sensor.rotation = YMPME_Rotation_Identity();
    sensor.time = feedback.time;
    sensor.valid = PME_FEEDBACK_POSE;
    scan.points = points;
    scan.capacity = 360;
    CHECK(YMPME_Lidar_Scan360(beams, 360, 360) == PME_OK);
    CHECK(YMPME_Lidar_Measure(&lidar, beams, 360, &sensor, &stamp, &provider, &seed, &scan) ==
              PME_OK &&
          points[0].valid && fabsf(points[0].range - 4.9f) < 1e-5f);
    CHECK(!points[180].valid);
    image.depth = depths;
    image.capacity = 9;
    CHECK(YMPME_Depth_BuildRays(&depth, rays, 9) == PME_OK);
    CHECK(YMPME_Depth_Measure(&depth, rays, 9, &sensor, &stamp, &provider, &seed, &image) ==
          PME_OK);
    CHECK(fabsf(depths[4] - points[0].range) < 1e-5f);
    CHECK(YMPME_World_Destroy(&w) == PME_OK);
    puts("Sensor scene: real world ray geometry and gravity/contact-driven tactile force passed");
    return 0;
}
