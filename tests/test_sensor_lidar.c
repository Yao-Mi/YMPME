#include "SENSOR/LIDAR/YMPME_Lidar.h"
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
typedef struct fixture_
{
    int count, fail;
    float distance;
    pme_ray last;
} fixture;
static pme_result cast(void *ctx, const pme_ray *ray, pme_ray_hit *out)
{
    fixture *f = ctx;
    f->last = *ray;
    ++f->count;
    if (f->fail && f->count == f->fail)
        return PME_INVALID_ARGUMENT;
    *out = (pme_ray_hit){1, f->distance, {0, 0, 0}, {0, 0, -1}, 7, SIZE_MAX};
    return PME_OK;
}
int main(void)
{
    pme_vec3 beams[360];
    pme_lidar_point points[360];
    pme_lidar_frame out = {0}, saved;
    pme_lidar_desc d = {YMPME_Sensor_IdentityMount(), .1f, 20, 0, 0};
    pme_feedback_state state = {0};
    pme_sensor_stamp stamp = {1, 1, 1, 1};
    uint32_t seed = 123;
    fixture f = {0, 0, 5, {{0, 0, 0}, {0, 0, 0}, 0}};
    pme_ray_provider provider = {cast, &f};
    size_t i;
    state.time = 1;
    state.valid = PME_FEEDBACK_POSE;
    state.rotation = YMPME_Rotation_Identity();
    out.points = points;
    out.capacity = 360;
    CHECK(YMPME_Lidar_Scan360(beams, 360, 360) == PME_OK);
    CHECK(fabsf(beams[0].z - 1) < 1e-6f && fabsf(beams[90].x - 1) < 1e-6f &&
          fabsf(beams[180].z + 1) < 1e-6f && fabsf(beams[270].x + 1) < 1e-6f);
    for (i = 0; i < 360; ++i)
        CHECK(beams[i].y == 0 && fabsf(YMPME_VecDot(beams[i], beams[i]) - 1) < 1e-6f);
    CHECK(YMPME_VecDot(beams[359], beams[0]) < .99999f);
    CHECK(YMPME_Lidar_Measure(&d, beams, 360, &state, &stamp, &provider, &seed, &out) == PME_OK &&
          out.count == 360);
    CHECK(points[90].valid && points[90].body == 7 && fabsf(points[90].point.x - 5) < 1e-5f);
    d.mount.offset = YMPME_Vec3(1, 2, 3);
    state.rotation = (pme_rotation){{{0, 0, -1}, {0, 1, 0}, {1, 0, 0}}};
    CHECK(YMPME_Lidar_Measure(&d, beams, 1, &state, &stamp, &provider, &seed, &out) == PME_OK);
    CHECK(f.last.origin.x == 3 && f.last.origin.z == -1 && f.last.direction.x == 1);
    f.distance = .05f;
    CHECK(YMPME_Lidar_Measure(&d, beams, 1, &state, &stamp, &provider, &seed, &out) == PME_OK &&
          !points[0].valid && isnan(points[0].range));
    f.distance = 5;
    d.noise = .1f;
    f.fail = f.count + 2;
    seed = 42;
    CHECK(YMPME_Lidar_Measure(&d, beams, 3, &state, &stamp, &provider, &seed, &out) ==
              PME_INVALID_ARGUMENT &&
          !out.stamp.valid && !out.count && seed == 42);
    f.fail = 0;
    saved = out;
    out.capacity = 1;
    saved = out;
    CHECK(YMPME_Lidar_Measure(&d, beams, 2, &state, &stamp, &provider, &seed, &out) ==
              PME_CAPACITY_EXCEEDED &&
          !memcmp(&saved, &out, sizeof(out)));
    puts("LiDAR: 2D 360 degree directions, ranges, mount transform, blind zone, capacity and "
         "provider failure passed");
    return 0;
}
