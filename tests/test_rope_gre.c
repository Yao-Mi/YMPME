#define _POSIX_C_SOURCE 200809L
#include "BRIDGE/YMPME_RopeGRE.h"
#include <stdio.h>
#include <string.h>
#include <time.h>
#define CHECK(c)                                                                                             \
    do                                                                                                       \
    {                                                                                                        \
        if (!(c))                                                                                            \
        {                                                                                                    \
            fprintf(stderr, "FAIL rope GRE line %d: %s\n", __LINE__, #c);                                    \
            return 1;                                                                                        \
        }                                                                                                    \
    } while (0)
#ifdef PME_TEST_ALLOC_WRAP
static size_t allocations;
static int tracking;
void *__real_malloc(size_t);
void *__real_calloc(size_t, size_t);
void *__real_realloc(void *, size_t);
void *__wrap_malloc(size_t n)
{
    if (tracking)
        ++allocations;
    return __real_malloc(n);
}
void *__wrap_calloc(size_t n, size_t s)
{
    if (tracking)
        ++allocations;
    return __real_calloc(n, s);
}
void *__wrap_realloc(void *p, size_t n)
{
    if (tracking)
        ++allocations;
    return __real_realloc(p, n);
}
#endif
static double now(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return t.tv_sec + t.tv_nsec * 1e-9;
}
static volatile double sink;
int main(void)
{
    pme_rope r;
    pme_rope_node nodes[33];
    pme_rope_link links[32];
    pme_rope_gre binding = {0};
    CHECK(YMPME_Rope_InitSpan(&r, nodes, 33, links, 32, 33, YMPME_Vec3(-1.5f, 1, 0), YMPME_Vec3(1.5f, 1, 0),
                              4, .04f, 1, 1) == PME_OK);
    CHECK(YMPME_GRE_Rope_Create(&binding, &r, 50, .025f, (GRErgb24){230, 180, 60}) == PME_OK);
    CHECK(YMPME_GRE_Rope_Sync(&binding, 1) == PME_OK && binding.bytesWritten > 0);
    GRE_Object4d object = binding.object;
    gre_vertex4d *vertices = object->pointList;
    void *indices = object->polygonList[0].index;
    uint64_t updates = binding.updates;
    CHECK(YMPME_GRE_Rope_Sync(&binding, 1) == PME_OK && binding.bytesWritten == 0 &&
          binding.updates == updates);
    CHECK(object->pointNum == 256 && object->polygonNum == 384);
    /* Each ring must average to its corresponding interpolated physics endpoint. */
#ifdef PME_TEST_ALLOC_WRAP
    tracking = 1;
#endif
    for (int i = 0; i < 120; ++i)
    {
        CHECK(YMPME_Rope_Step(&r, 1.0f / 120) == PME_OK);
        CHECK(YMPME_GRE_Rope_Sync(&binding, .5f) == PME_OK);
    }
#ifdef PME_TEST_ALLOC_WRAP
    tracking = 0;
    CHECK(allocations == 0);
#endif
    for (size_t edge = 0; edge < r.linkCount; ++edge)
        for (int end = 0; end < 2; ++end)
        {
            pme_vec3 center = YMPME_Vec3(0, 0, 0);
            for (int k = 0; k < 4; ++k)
            {
                gre_fvector4d v = object->pointList[edge * 8 + end * 4 + k].pos;
                center = YMPME_VecAdd(center, YMPME_Vec3(v.x, v.y, v.z));
            }
            center = YMPME_VecScale(center, .25f);
            const pme_rope_node *n = &nodes[edge + end];
            pme_vec3 expected = YMPME_VecScale(YMPME_VecAdd(n->position, n->previous), 25);
            pme_vec3 error = YMPME_VecSub(center, expected);
            CHECK(YMPME_VecDot(error, error) < 1e-7f);
        }
    CHECK(vertices == object->pointList && indices == object->polygonList[0].index && binding.nodes == nodes);
    gre_vertex4d copy[256];
    memcpy(copy, vertices, sizeof(copy));
    CHECK(YMPME_GRE_Rope_Sync(&binding, NAN) == PME_INVALID_ARGUMENT);
    CHECK(!memcmp(copy, vertices, sizeof(copy)));
    /* Direct array access versus borrowed view: same interpolation reduction, no copy in either path. */
    double direct = 0, viewTime = 0;
    const int runs = 30000;
    for (int pass = 0; pass < 4; ++pass)
    {
        double start = now(), sum = 0;
        for (int k = 0; k < runs; ++k)
            for (size_t i = 0; i < r.count; ++i)
                sum += (nodes[i].position.y + nodes[i].previous.y) * .5;
        direct += now() - start;
        sink = sum;
        start = now();
        sum = 0;
        for (int k = 0; k < runs; ++k)
        {
            pme_rope_view v = YMPME_Rope_GetView(&r);
            for (size_t i = 0; i < v.count; ++i)
                sum += (v.nodes[i].position.y + v.nodes[i].previous.y) * .5;
        }
        viewTime += now() - start;
        sink = sum;
    }
    double start = now();
    for (int i = 0; i < 20000; ++i)
        CHECK(YMPME_GRE_Rope_Sync(&binding, (i % 2) * .5f) == PME_OK);
    double sync = (now() - start) / 20000;
    start = now();
    for (int i = 0; i < 1200; ++i)
        CHECK(YMPME_Rope_Step(&r, 1.0f / 120) == PME_OK);
    double physics = (now() - start) / 1200;
    printf("BENCH 33 nodes / 256 vertices: direct read %.3f us, borrowed view %.3f us, mesh sync %.3f us, "
           "physics(1/120s) %.3f us; writes=%zu bytes/sync\n",
           direct / (4 * runs) * 1e6, viewTime / (4 * runs) * 1e6, sync * 1e6, physics * 1e6,
           binding.bytesWritten);
    /* Reset at identical addresses must invalidate the binding cache without reallocating. */
    CHECK(YMPME_Rope_InitSpan(&r, nodes, 33, links, 32, 33, YMPME_Vec3(-1.5f, 1, 0), YMPME_Vec3(1.5f, 1, 0),
                              4, .04f, 1, 1) == PME_OK);
    CHECK(YMPME_GRE_Rope_Rebind(&binding, &r) == PME_OK);
    CHECK(YMPME_GRE_Rope_Sync(&binding, 1) == PME_OK && binding.bytesWritten > 0);
    CHECK(binding.object->pointList == vertices);
    YMPME_GRE_Rope_Destroy(&binding);
    CHECK(binding.object == NULL);
    YMPME_GRE_Rope_Destroy(&binding);
    puts("PASS rope GRE: endpoint interpolation, stable buffers, cached frames, invalid input");
    return 0;
}
