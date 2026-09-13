#include "SENSOR/DEPTH/YMPME_Depth.h"
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
static pme_result plane(void *ctx, const pme_ray *ray, pme_ray_hit *out)
{
    float z = *(float *)ctx;
    *out = (pme_ray_hit){1,       (z - ray->origin.z) / ray->direction.z, {0, 0, 0}, {0, 0, -1}, 1,
                         SIZE_MAX};
    if (out->distance > ray->maxDistance)
        out->hit = 0;
    return PME_OK;
}
int main(void)
{
    pme_depth_desc d = {YMPME_Sensor_IdentityMount(), 3, 3, 1, 1, 1.5f, 1.5f, .1f, 20, 0, 0};
    pme_vec3 rays[9], points[9];
    float depth[9], z = 5;
    pme_depth_frame out = {0};
    uint16_t mm[9];
    pme_feedback_state s = {0};
    pme_sensor_stamp stamp = {1, 1, 1, 1};
    uint32_t seed = 1;
    size_t i;
    pme_ray_provider provider = {plane, &z};
    s.time = 1;
    s.valid = PME_FEEDBACK_POSE;
    s.rotation = YMPME_Rotation_Identity();
    out.depth = depth;
    out.capacity = 9;
    CHECK(YMPME_Depth_BuildRays(&d, rays, 9) == PME_OK);
    CHECK(rays[0].x < 0 && rays[0].y > 0 && rays[4].z == 1);
    CHECK(YMPME_Depth_Measure(&d, rays, 9, &s, &stamp, &provider, &seed, &out) == PME_OK);
    /* A fronto-parallel plane has equal axial depth even in the corners. */
    for (i = 0; i < 9; ++i)
        CHECK(fabsf(depth[i] - 5) < 1e-5f);
    CHECK(YMPME_Depth_Unproject(&out, rays, 9, points, 9) == PME_OK);
    CHECK(fabsf(points[0].x + 5) < 1e-5f && fabsf(points[0].y - 5) < 1e-5f);
    CHECK(YMPME_Depth_EncodeU16(&out, 1000, mm, 9) == PME_OK && mm[0] == 5000);
    depth[0] = NAN;
    depth[1] = 100;
    CHECK(YMPME_Depth_EncodeU16(&out, 1000, mm, 9) == PME_OK && mm[0] == 0 && mm[1] == 0);
    CHECK(YMPME_Depth_Unproject(&out, rays, 9, points, 9) == PME_OK && isnan(points[0].x));
    z = .05f;
    CHECK(YMPME_Depth_Measure(&d, rays, 9, &s, &stamp, &provider, &seed, &out) == PME_OK &&
          isnan(depth[4]));
    z = 30;
    CHECK(YMPME_Depth_Measure(&d, rays, 9, &s, &stamp, &provider, &seed, &out) == PME_OK &&
          isnan(depth[4]));
    z = 5;
    d.fx = 2;
    uint32_t savedSeed = seed;
    float savedDepth = depth[4];
    CHECK(YMPME_Depth_Measure(&d, rays, 9, &s, &stamp, &provider, &seed, &out) ==
          PME_INVALID_ARGUMENT);
    CHECK(seed == savedSeed && isnan(savedDepth) && isnan(depth[4]));
    CHECK(YMPME_Depth_BuildRays(&d, rays, 9) == PME_OK);
    CHECK(YMPME_Depth_Measure(&d, rays, 9, &s, &stamp, &provider, &seed, &out) == PME_OK &&
          fabsf(depth[4] - 5) < 1e-5f);
    rays[8].z = -1;
    CHECK(YMPME_Depth_Measure(&d, rays, 9, &s, &stamp, &provider, &seed, &out) ==
          PME_INVALID_ARGUMENT);
    puts("Depth: pixel convention, axial depth vs ray distance, unprojection, uint16 mm, clipping "
         "passed");
    return 0;
}
