#include "CORE/QUERY/YMPME_Query.h"
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
#define V YMPME_Vec3
int main(void)
{
    pme_world w;
    pme_body bodies[6];
    pme_body_desc d = YMPME_Body_DefaultDesc();
    pme_world_query q;
    pme_ray ray = {V(0, 0, 0), V(0, 0, 1), 100};
    pme_ray_hit hit, saved;
    const pme_vec3 vertices[] = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    const uint32_t indices[] = {0, 2, 1, 0, 1, 3, 0, 3, 2, 1, 2, 3};
    pme_vec3 faces[4], edges[12];
    pme_convex_mesh mesh;
    CHECK(YMPME_World_Init(&w, bodies, 6, NULL) == PME_OK);
    q = (pme_world_query){&w, YMPME_Query_DefaultFilter()};
    d.type = PME_STATIC;
    d.shape = PME_SPHERE;
    d.radius = 1;
    d.position = V(0, 0, 5);
    CHECK(YMPME_World_AddBody(&w, &d));
    d.shape = PME_BOX;
    d.halfExtents = V(1, 1, 1);
    d.position = V(0, 0, 9);
    CHECK(YMPME_World_AddBody(&w, &d));
    CHECK(YMPME_Query_WorldRaycast(&q, &ray, &hit) == PME_OK && hit.hit && hit.body == 0 &&
          fabsf(hit.distance - 4) < 1e-5f);
    CHECK(fabsf(hit.normal.z + 1) < 1e-5f);
    q.filter.ignoreBody = 0;
    CHECK(YMPME_Query_WorldRaycast(&q, &ray, &hit) == PME_OK && hit.body == 1 &&
          fabsf(hit.distance - 8) < 1e-5f);
    q.filter.ignoreBody = SIZE_MAX;
    ray.origin = V(0, 0, 5);
    CHECK(YMPME_Query_WorldRaycast(&q, &ray, &hit) == PME_OK && fabsf(hit.distance - 1) < 1e-5f);
    ray.origin = V(1, 0, 0);
    CHECK(YMPME_Query_WorldRaycast(&q, &ray, &hit) == PME_OK && fabsf(hit.distance - 5) < 1e-5f);
    ray.origin = V(0, 0, 0);
    ray.maxDistance = 3;
    CHECK(YMPME_Query_WorldRaycast(&q, &ray, &hit) == PME_OK && !hit.hit);
    ray.maxDistance = 100;
    bodies[0].state.sensor = 1;
    CHECK(YMPME_Query_WorldRaycast(&q, &ray, &hit) == PME_OK && hit.body == 1);
    q.filter.includeSensors = 1;
    CHECK(YMPME_Query_WorldRaycast(&q, &ray, &hit) == PME_OK && hit.body == 0);
    q.filter.ignoreAssembly = bodies[0].state.ignoreAssembly = 7;
    CHECK(YMPME_Query_WorldRaycast(&q, &ray, &hit) == PME_OK && hit.body == 1);
    q.filter.ignoreAssembly = 0;
    bodies[0].state.collisionGroup = 3;
    CHECK(YMPME_World_SetGroupCollision(&w, 0, 3, 0) == PME_OK);
    CHECK(YMPME_Query_WorldRaycast(&q, &ray, &hit) == PME_OK && hit.body == 1);
    CHECK(YMPME_World_SetGroupCollision(&w, 0, 3, 1) == PME_OK);
    q.filter.mask = 2;
    bodies[0].state.category = 1;
    bodies[1].state.category = 2;
    CHECK(YMPME_Query_WorldRaycast(&q, &ray, &hit) == PME_OK && hit.body == 1);
    CHECK(YMPME_ConvexMesh_Init(&mesh, vertices, 4, indices, 4, faces, 4, edges, 12) == PME_OK);
    CHECK(YMPME_World_Destroy(&w) == PME_OK);
    CHECK(YMPME_World_Init(&w, bodies, 6, NULL) == PME_OK);
    q.filter = YMPME_Query_DefaultFilter();
    d.shape = PME_CONVEX_MESH;
    d.mesh = &mesh;
    d.position = V(0, 0, 0);
    d.rotation = (pme_rotation){{{0, 0, -1}, {0, 1, 0}, {1, 0, 0}}};
    CHECK(YMPME_World_AddBody(&w, &d));
    ray.origin = V(-1, .2f, -.2f);
    ray.direction = V(1, 0, 0);
    ray.maxDistance = 3;
    CHECK(YMPME_Query_WorldRaycast(&q, &ray, &hit) == PME_OK && hit.hit &&
          fabsf(hit.distance - 1) < 1e-5f);
    CHECK(hit.normal.x < -.99f && hit.primitive < 4);
    /* AABB hit but triangle miss, independent of collision precision toggle. */
    ray.origin = V(-1, .8f, -.8f);
    w.config.meshPrecision = 0;
    CHECK(YMPME_Query_WorldRaycast(&q, &ray, &hit) == PME_OK && !hit.hit);
    /* Internal ray reaches actual surface before exiting the enclosing AABB. */
    ray.origin = V(.2f, .2f, -.2f);
    ray.maxDistance = .5f;
    CHECK(YMPME_Query_WorldRaycast(&q, &ray, &hit) == PME_OK && hit.hit &&
          fabsf(hit.distance - .4f) < 1e-5f);
    saved = hit;
    ray.direction = V(2, 0, 0);
    CHECK(YMPME_Query_WorldRaycast(&q, &ray, &hit) == PME_INVALID_ARGUMENT &&
          !memcmp(&hit, &saved, sizeof(hit)));
    CHECK(YMPME_World_Destroy(&w) == PME_OK);
    puts("Ray query: sphere/box/rotated mesh, inside, tangent, range, self/mask/group/assembly "
         "filters passed");
    return 0;
}
