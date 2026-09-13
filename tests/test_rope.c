#include "MODEL/ROPE/YMPME_Rope.h"
#include <stdio.h>
#include <string.h>
#define CHECK(c)                                                                                             \
    do                                                                                                       \
    {                                                                                                        \
        if (!(c))                                                                                            \
        {                                                                                                    \
            fprintf(stderr, "FAIL rope line %d: %s\n", __LINE__, #c);                                        \
            return 1;                                                                                        \
        }                                                                                                    \
    } while (0)
static double distance(pme_vec3 a, pme_vec3 b)
{
    pme_vec3 d = YMPME_VecSub(a, b);
    return sqrt(YMPME_VecDot(d, d));
}
static double stretch(const pme_rope *r)
{
    double worst = 0;
    for (size_t i = 0; i < r->linkCount; ++i)
        worst = fmax(worst, distance(r->nodes[i].position, r->nodes[(i + 1) % r->count].position) /
                                    r->links[i].restLength -
                                1);
    return worst;
}
int main(void)
{
    pme_rope r, other;
    pme_rope_node n[33], backup[33], n2[33];
    pme_rope_link l[33], l2[33];
    CHECK(YMPME_Rope_InitSpan(&r, n, 33, l, 33, 33, YMPME_Vec3(-1.5f, 1, 0), YMPME_Vec3(1.5f, 1, 0), 4, .04f,
                              1, 1) == PME_OK);
    CHECK(YMPME_Rope_InitSpan(&other, n2, 33, l2, 33, 2, YMPME_Vec3(0, 1, 0), YMPME_Vec3(1.2f, -.6f, 0), 2, 2,
                              1, 0) == PME_OK);
    CHECK(!memcmp(&n2[1].position, &n2[1].previous, sizeof(pme_vec3)) &&
          YMPME_VecDot(n2[1].velocity, n2[1].velocity) == 0);
    r.damping = 2;
    r.iterations = 48;
    other.damping = 0;
    double left = 10, right = -10, e0 = 2 * 9.81 * (n2[1].position.y + 1), worst = 0;
    for (int i = 0; i < 1200; ++i)
    {
        CHECK(YMPME_Rope_Step(&r, 1.0f / 120) == PME_OK);
        CHECK(YMPME_Rope_Step(&other, 1.0f / 120) == PME_OK);
        CHECK(n[0].position.x == -1.5f && n[0].position.y == 1 && n[32].position.x == 1.5f &&
              n[32].position.y == 1);
        CHECK(n2[0].position.x == 0 && n2[0].position.y == 1);
        double energy = YMPME_VecDot(n2[1].velocity, n2[1].velocity) + 2 * 9.81 * (n2[1].position.y + 1);
        CHECK(energy < e0 * 1.03);
        CHECK(distance(n2[0].position, n2[1].position) <= 2.000002);
        left = fmin(left, n2[1].position.x);
        right = fmax(right, n2[1].position.x);
        worst = fmax(worst, stretch(&r));
    }
    printf("sag=%.4fm stretch=%.4f%% swing=[%.3f,%.3f]\n", 1 - n[16].position.y, worst * 100, left, right);
    CHECK(n[16].position.y < 0 && fabs(n[16].position.x) < .04 && worst < .035);
    CHECK(left < -1 && right > 1);
    /* Borrowed view is exactly the caller buffer; invalid calls do not alter it. */
    pme_rope_view view = YMPME_Rope_GetView(&r);
    CHECK(view.nodes == n && view.links == l && view.count == 33);
    memcpy(backup, n, sizeof(n));
    pme_rope old = r;
    CHECK(YMPME_Rope_Step(&r, NAN) == PME_INVALID_ARGUMENT && YMPME_Rope_Step(&r, 0) == PME_INVALID_ARGUMENT);
    CHECK(YMPME_Rope_InitSpan(&r, n, 1, l, 33, 33, YMPME_Vec3(0, 0, 0), YMPME_Vec3(1, 0, 0), 2, 1, 1, 1) ==
          PME_CAPACITY_EXCEEDED);
    CHECK(!memcmp(backup, n, sizeof(n)) && !memcmp(&old, &r, sizeof(r)));
    CHECK(YMPME_Rope_SetAnchor(&r, 1, YMPME_Vec3(1, 1, 1)) == PME_INVALID_ARGUMENT);
    CHECK(YMPME_Rope_AddImpulse(&r, 0, YMPME_Vec3(1, 0, 0)) == PME_INVALID_ARGUMENT);
    for (int i = 0; i < 240; ++i)
    {
        float x = .25f * sinf(i / 60.0f);
        CHECK(YMPME_Rope_SetAnchor(&r, 0, YMPME_Vec3(-1.5f + x, 1, 0)) == PME_OK);
        CHECK(YMPME_Rope_SetAnchor(&r, 32, YMPME_Vec3(1.5f + x, 1, 0)) == PME_OK);
        CHECK(YMPME_Rope_Step(&r, 1.0f / 120) == PME_OK);
        CHECK(fabs(n[0].position.x + 1.5f - x) < 1e-6 && stretch(&r) < .035);
    }
    CHECK(YMPME_Rope_SetMass(&r, 32, .04f) == PME_OK);
    float endY = n[32].position.y;
    for (int i = 0; i < 60; ++i)
        CHECK(YMPME_Rope_Step(&r, 1.0f / 120) == PME_OK);
    CHECK(n[32].position.y < endY - .1f);
    /* A slack two-node rope falls freely and cannot push its mass. */
    CHECK(YMPME_Rope_InitSpan(&r, n, 33, l, 33, 2, YMPME_Vec3(0, 0, 0), YMPME_Vec3(0, -1, 0), 2, 2, 1, 0) ==
          PME_OK);
    CHECK(YMPME_Rope_Step(&r, .01f) == PME_OK && l[0].tension == 0 && n[1].position.y < -1);
    /* Static load: tension is m*g. */
    CHECK(YMPME_Rope_InitSpan(&r, n, 33, l, 33, 2, YMPME_Vec3(0, 0, 0), YMPME_Vec3(0, -2, 0), 2, 2, 1, 0) ==
          PME_OK);
    for (int i = 0; i < 120; ++i)
        CHECK(YMPME_Rope_Step(&r, 1.0f / 120) == PME_OK);
    printf("static tension %.5f N\n", l[0].tension);
    CHECK(fabs(l[0].tension - 19.62) < .3);
    r.compliance = .002f;
    r.damping = 3;
    for (int i = 0; i < 720; ++i)
        CHECK(YMPME_Rope_Step(&r, 1.0f / 120) == PME_OK);
    CHECK(fabs(distance(n[0].position, n[1].position) - 2 - .002 * 19.62) < .002);
    old = r;
    r.maxSubstep = 0;
    CHECK(YMPME_Rope_Step(&r, 1.0f / 120) == PME_INVALID_ARGUMENT);
    r = old;
    /* Descriptor validates every element before writing; closed loop includes the last edge. */
    pme_vec3 points[4] = {{-1, 0, -1}, {1, 0, -1}, {1, 0, 1}, {-1, 0, 1}};
    float masses[4] = {1, 1, 1, 1};
    pme_rope_desc d = YMPME_Rope_DefaultDesc();
    d.count = 4;
    d.closed = 1;
    d.positions = points;
    d.masses = masses;
    d.gravity = YMPME_Vec3(0, 0, 0);
    CHECK(YMPME_Rope_Init(&r, &d, n, 33, l, 33) == PME_OK && r.linkCount == 4);
    CHECK(YMPME_Rope_AddImpulse(&r, 0, YMPME_Vec3(0, 1, 0)) == PME_OK);
    CHECK(YMPME_Rope_AddImpulse(&r, 2, YMPME_Vec3(0, -1, 0)) == PME_OK);
    for (int i = 0; i < 240; ++i)
        CHECK(YMPME_Rope_Step(&r, 1.0f / 120) == PME_OK && stretch(&r) < .005);
    pme_vec3 center = YMPME_Vec3(0, 0, 0);
    for (int i = 0; i < 4; ++i)
        center = YMPME_VecAdd(center, n[i].position);
    center = YMPME_VecScale(center, .25f);
    CHECK(distance(center, YMPME_Vec3(0, 0, 0)) < .001);
    memcpy(backup, n, sizeof(n));
    old = r;
    masses[3] = -1;
    CHECK(YMPME_Rope_Init(&r, &d, n, 33, l, 33) == PME_INVALID_ARGUMENT);
    CHECK(!memcmp(backup, n, sizeof(n)) && !memcmp(&old, &r, sizeof(r)));
    masses[3] = 1;
    d.positions = &n[0].position;
    CHECK(YMPME_Rope_Init(&r, &d, n, 33, l, 33) == PME_INVALID_ARGUMENT);
    CHECK(YMPME_Rope_GetView(NULL).count == 0);
    puts("PASS rope: sag, swing, slack, tension, moving/released anchors, closed loop, borrowed view and "
         "rejected inputs");
    return 0;
}
