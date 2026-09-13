#include "YMPME.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define CHECK(x) do { if (!(x)) { fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #x); return 1; } } while (0)
#define NEAR(a,b,e) CHECK(fabs((double)(a)-(double)(b)) <= (e))
/* 替换平台内存接口，验证失败回滚和热路径零分配。 */
static size_t allocations, frees;
static int failAfter = -1;
void* PME_malloc0(size_t bytes)
{
    void* p;
    if (failAfter == 0) return NULL;
    if (failAfter > 0) --failAfter;
    p = malloc(bytes);
    if (p) ++allocations;
    return p;
}
void PME_free0(void* p)
{
    if (p) ++frees;
    free(p);
}
static int integration(void)
{
    pme_world w;
    pme_body slots[4];
    pme_body_desc d = YMPME_Body_DefaultDesc();
    PME_Body b, k, s;
    int i;
    CHECK(YMPME_World_Init(&w, slots, 4, NULL) == PME_OK);
    d.position.y = 10;
    d.mask = 0;
    b = YMPME_World_AddBody(&w, &d);
    CHECK(b);
    d.type = PME_KINEMATIC;
    d.velocity.x = 2;
    k = YMPME_World_AddBody(&w, &d);
    d.type = PME_STATIC;
    s = YMPME_World_AddBody(&w, &d);
    CHECK(k && s);
    for (i = 0; i < 120; ++i) CHECK(YMPME_World_Step(&w, 1.0f/120) == PME_OK);
    NEAR(b->state.velocity.y, -9.81, 0.00002);
    NEAR(b->state.position.y, 10 - 0.5*9.81*(1+1.0/120), 0.00002);
    NEAR(k->state.position.x, 2, 0.00001);
    NEAR(k->state.position.y, 10, 0.00001);
    NEAR(s->state.position.x, 0, 0);
    w.config.gravity = YMPME_Vec3(0, 0, 0);
    b->state.velocity = YMPME_Vec3(0, 0, 0);
    CHECK(YMPME_Body_AddForce(b, YMPME_Vec3(2, 0, 0)) == PME_OK);
    CHECK(YMPME_Body_AddImpulse(b, YMPME_Vec3(3, 0, 0)) == PME_OK);
    CHECK(YMPME_World_Step(&w, 0.5f) == PME_OK);
    NEAR(b->state.velocity.x, 4, 1e-6);
    NEAR(b->force.x, 0, 0);
    b->state.linearDamping = 2;
    CHECK(YMPME_World_Step(&w, 0.5f) == PME_OK);
    NEAR(b->state.velocity.x, 2, 1e-6);
    CHECK(YMPME_Body_SetPosition(b, YMPME_Vec3(5, 6, 7)) == PME_OK);
    NEAR(YMPME_Body_InterpolatedPosition(b, 0).x, 5, 0);
    return 0;
}
static int collision(void)
{
    pme_body a, b;
    pme_body_desc d = YMPME_Body_DefaultDesc();
    pme_contact c, reverse;
    int axis, sign;
    CHECK(YMPME_Body_Init(&a, &d) == PME_OK);
    d.position.x = 1;
    CHECK(YMPME_Body_Init(&b, &d) == PME_OK);
    CHECK(YMPME_Collide(&a, &b, &c));
    NEAR(c.penetration, 0, 0);
    NEAR(c.normal.x, 1, 0);
    b.state.position.x = 1.01f;
    CHECK(!YMPME_Collide(&a, &b, &c));
    b.state.position.x = 0;
    CHECK(YMPME_Collide(&a, &b, &c));
    NEAR(c.penetration, 1, 0);
    CHECK(YMPME_VecFinite(c.normal));
    b.state.shape = PME_BOX;
    b.state.halfExtents = YMPME_Vec3(1, 1, 1);
    /* 球心位于盒内、表面、外侧，覆盖六个面的法线与反序。 */
    for (axis = 0; axis < 3; ++axis)
    for (sign = -1; sign <= 1; sign += 2)
    {
        float distance[] = {0.75f, 1.0f, 1.25f};
        size_t i;
        for (i = 0; i < 3; ++i)
        {
            pme_vec3 p = {0, 0, 0};
            if (axis == 0) p.x = distance[i]*sign;
            if (axis == 1) p.y = distance[i]*sign;
            if (axis == 2) p.z = distance[i]*sign;
            a.state.position = p;
            CHECK(YMPME_Collide(&a, &b, &c));
            CHECK(YMPME_Collide(&b, &a, &reverse));
            NEAR(c.penetration, 1.5f-distance[i], 1e-6);
            CHECK(YMPME_VecDot(c.normal, p) < 0);
            NEAR(YMPME_VecDot(c.normal, reverse.normal), -1, 1e-6);
        }
    }
    a.state.position = YMPME_Vec3(1.4f, 1.4f, 0);
    CHECK(!YMPME_Collide(&a, &b, NULL));
    a.state.position = YMPME_Vec3(1.3f, 1.3f, 0);
    CHECK(YMPME_Collide(&a, &b, &c));
    NEAR(c.penetration, 0.5-sqrt(0.18), 1e-6);
    a.state.shape = PME_BOX;
    a.state.position = YMPME_Vec3(0, 1.25f, 0);
    CHECK(YMPME_Collide(&a, &b, &c));
    NEAR(c.normal.y, -1, 0);
    NEAR(c.penetration, 0.25, 1e-6);
    a.state.position = YMPME_Vec3(0, 0, 1.25f);
    CHECK(YMPME_Collide(&a, &b, &c));
    NEAR(c.normal.z, -1, 0);
    a.state.position.z = 2;
    CHECK(!YMPME_Collide(&a, &b, NULL));
    return 0;
}
static int response(void)
{
    pme_world w;
    pme_body slots[2];
    pme_world_desc cfg = YMPME_World_DefaultDesc();
    pme_body_desc d = YMPME_Body_DefaultDesc();
    PME_Body a, b;
    double energy;
    cfg.gravity = YMPME_Vec3(0, 0, 0);
    CHECK(YMPME_World_Init(&w, slots, 2, &cfg) == PME_OK);
    d.position.x = -0.5f;
    d.velocity.x = 2;
    d.restitution = 1;
    d.friction = 0;
    a = YMPME_World_AddBody(&w, &d);
    d.position.x = 0.5f;
    d.velocity.x = -1;
    d.mass = 2;
    b = YMPME_World_AddBody(&w, &d);
    CHECK(a && b);
    energy = YMPME_Debug_KineticEnergy(&w);
    CHECK(YMPME_World_Step(&w, 0.001f) == PME_OK);
    NEAR(a->state.velocity.x, -2, 1e-5);
    NEAR(b->state.velocity.x, 1, 1e-5);
    NEAR(a->state.velocity.x+2*b->state.velocity.x, 0, 1e-5);
    NEAR(YMPME_Debug_KineticEnergy(&w), energy, 1e-5);
    /* 运动平台推动动态球，平台速度不变。 */
    CHECK(YMPME_World_Destroy(&w) == PME_OK);
    CHECK(YMPME_World_Init(&w, slots, 2, &cfg) == PME_OK);
    d = YMPME_Body_DefaultDesc();
    d.type = PME_KINEMATIC;
    d.velocity.x = 1;
    a = YMPME_World_AddBody(&w, &d);
    d.type = PME_DYNAMIC;
    d.velocity.x = 0;
    d.position.x = 1;
    b = YMPME_World_AddBody(&w, &d);
    CHECK(a && b);
    CHECK(YMPME_World_Step(&w, 0.01f) == PME_OK);
    NEAR(a->state.velocity.x, 1, 0);
    NEAR(b->state.velocity.x, 1, 1e-6);
    return 0;
}
static int resting(void)
{
    pme_world w;
    pme_body slots[3];
    pme_body_desc d = YMPME_Body_DefaultDesc();
    PME_Body ball, box;
    int i;
    CHECK(YMPME_World_Init(&w, slots, 3, NULL) == PME_OK);
    d.type = PME_STATIC;
    d.shape = PME_BOX;
    d.halfExtents = YMPME_Vec3(100, 0.5f, 100);
    d.position.y = -0.5f;
    CHECK(YMPME_World_AddBody(&w, &d));
    d = YMPME_Body_DefaultDesc();
    d.position = YMPME_Vec3(0, 3, 0);
    d.velocity.x = 3;
    ball = YMPME_World_AddBody(&w, &d);
    d.shape = PME_BOX;
    d.position.x = -5;
    d.velocity.x = 0;
    box = YMPME_World_AddBody(&w, &d);
    CHECK(ball && box);
    for (i = 0; i < 2400; ++i) CHECK(YMPME_World_Step(&w, w.config.fixedStep) == PME_OK);
    NEAR(ball->state.position.y, 0.5, 0.001);
    NEAR(box->state.position.y, 0.5, 0.001);
    NEAR(ball->state.velocity.y, 0, 1e-6);
    NEAR(ball->state.velocity.x, 0, 1e-6);
    NEAR(box->state.velocity.y, 0, 1e-6);
    return 0;
}
static int lifecycle(void)
{
    pme_world w = {0};
    pme_body slots[1], foreign, saved;
    pme_body_desc d = YMPME_Body_DefaultDesc();
    pme_world_desc cfg = YMPME_World_DefaultDesc();
    PME_Body b;
    CHECK(YMPME_World_Init(NULL, slots, 1, NULL) == PME_INVALID_ARGUMENT);
    CHECK(YMPME_World_Init(&w, slots, 0, NULL) == PME_INVALID_ARGUMENT);
    cfg.fixedStep = NAN;
    CHECK(YMPME_World_Init(&w, slots, 1, &cfg) == PME_INVALID_ARGUMENT);
    CHECK(YMPME_World_Init(&w, slots, 1, NULL) == PME_OK);
    d.mass = 0;
    CHECK(!YMPME_World_AddBody(&w, &d));
    d.mass = 1;
    b = YMPME_World_AddBody(&w, &d);
    CHECK(b && w.bodyCount == 1);
    CHECK(!YMPME_World_AddBody(&w, &d));
    saved = *b;
    d.radius = NAN;
    CHECK(YMPME_Body_Init(b, &d) == PME_INVALID_ARGUMENT);
    CHECK(memcmp(b, &saved, sizeof(saved)) == 0);
    CHECK(YMPME_Body_AddForce(b, YMPME_Vec3(NAN, 0, 0)) == PME_INVALID_ARGUMENT);
    CHECK(YMPME_Body_AddImpulse(b, YMPME_Vec3(INFINITY, 0, 0)) == PME_INVALID_ARGUMENT);
    CHECK(YMPME_World_Step(&w, -1) == PME_INVALID_ARGUMENT);
    CHECK(YMPME_World_Step(&w, NAN) == PME_INVALID_ARGUMENT);
    CHECK(YMPME_World_Advance(&w, INFINITY) == PME_INVALID_ARGUMENT);
    CHECK(YMPME_World_RemoveBody(&w, &foreign) == PME_INVALID_ARGUMENT);
    CHECK(YMPME_World_RemoveBody(&w, b) == PME_OK);
    CHECK(YMPME_World_RemoveBody(&w, b) == PME_INVALID_ARGUMENT);
    CHECK(w.bodyCount == 0);
    d = YMPME_Body_DefaultDesc();
    CHECK(YMPME_World_AddBody(&w, &d) == b);
    CHECK(YMPME_World_Destroy(&w) == PME_OK);
    CHECK(!w.bodies && !slots[0].active);
    return 0;
}
static int fixedStep(void)
{
    pme_world a, b;
    pme_body sa[1], sb[1];
    pme_body_desc d = YMPME_Body_DefaultDesc();
    PME_Body ba, bb;
    int i;
    CHECK(YMPME_World_Init(&a, sa, 1, NULL) == PME_OK);
    CHECK(YMPME_World_Init(&b, sb, 1, NULL) == PME_OK);
    ba = YMPME_World_AddBody(&a, &d);
    bb = YMPME_World_AddBody(&b, &d);
    CHECK(ba && bb);
    for (i = 0; i < 120; ++i) CHECK(YMPME_World_Step(&a, a.config.fixedStep) == PME_OK);
    for (i = 0; i < 60; ++i) CHECK(YMPME_World_Advance(&b, 1.0f/60) == 2);
    NEAR(ba->state.position.y, bb->state.position.y, 0);
    NEAR(ba->state.velocity.y, bb->state.velocity.y, 0);
    CHECK(YMPME_World_Advance(&b, b.config.fixedStep*0.5f) == 0);
    NEAR(YMPME_World_InterpolationAlpha(&b), 0.5, 1e-6);
    CHECK(YMPME_World_Advance(&b, 2) == 8);
    CHECK(b.droppedTime > 1.9);
    CHECK(b.accumulator >= 0 && b.accumulator < b.config.fixedStep);
    a.config.gravity = YMPME_Vec3(0, 0, 0);
    ba->state.velocity = YMPME_Vec3(0, 0, 0);
    CHECK(YMPME_Body_AddForce(ba, YMPME_Vec3(6, 0, 0)) == PME_OK);
    CHECK(YMPME_World_Advance(&a, a.config.fixedStep*0.5f) == 0);
    NEAR(ba->force.x, 6, 0);
    CHECK(YMPME_World_Advance(&a, a.config.fixedStep*1.5f) == 2);
    NEAR(ba->state.velocity.x, 6*2*a.config.fixedStep, 1e-6);
    NEAR(ba->force.x, 0, 0);
    return 0;
}
static int filtering(void)
{
    pme_world w;
    pme_body slots[2];
    pme_body_desc d = YMPME_Body_DefaultDesc();
    PME_Body a, b;
    CHECK(YMPME_World_Init(&w, slots, 2, NULL) == PME_OK);
    w.config.gravity = YMPME_Vec3(0, 0, 0);
    d.type = PME_STATIC;
    d.sensor = 1;
    a = YMPME_World_AddBody(&w, &d);
    d.type = PME_DYNAMIC;
    d.sensor = 0;
    d.velocity.x = 1;
    b = YMPME_World_AddBody(&w, &d);
    CHECK(a && b && YMPME_Collide(a, b, NULL));
    CHECK(YMPME_World_Step(&w, 0.1f) == PME_OK);
    NEAR(b->state.position.x, 0.1, 1e-6);
    NEAR(b->state.velocity.x, 1, 0);
    a->state.mask = 2;
    CHECK(!YMPME_Collide(a, b, NULL));
    a->state.mask = UINT32_MAX;
    b->state.mask = 0;
    CHECK(!YMPME_Collide(a, b, NULL));
    CHECK(!YMPME_Collide(a, a, NULL));
    return 0;
}
static int memory(void)
{
    pme_world_desc cfg = YMPME_World_DefaultDesc();
    pme_body_desc d = YMPME_Body_DefaultDesc();
    PME_World w;
    size_t start;
    int i;
    failAfter = 0;
    CHECK(!YMPME_Create_World(4, NULL));
    failAfter = 1;
    CHECK(!YMPME_Create_World(4, NULL));
    CHECK(allocations == frees);
    failAfter = -1;
    cfg.fixedStep = -1;
    CHECK(!YMPME_Create_World(4, &cfg));
    CHECK(allocations == frees);
    CHECK(!YMPME_Create_World(SIZE_MAX, NULL));
    w = YMPME_Create_World(4, NULL);
    CHECK(w && YMPME_World_AddBody(w, &d));
    CHECK(YMPME_World_Destroy(w) == PME_INVALID_ARGUMENT);
    start = allocations;
    for (i = 0; i < 100; ++i) CHECK(YMPME_World_Advance(w, 1.0f/60) == 2);
    CHECK(allocations == start);
    YMPME_Free_World(w);
    YMPME_Free_World(NULL);
    CHECK(allocations == frees);
    return 0;
}
int main(int argc, char** argv)
{
    struct { const char* name; int (*run)(void); } tests[] = {
        {"integration", integration}, {"collision", collision}, {"response", response},
        {"resting", resting}, {"lifecycle", lifecycle}, {"fixed_step", fixedStep},
        {"filtering", filtering}, {"memory", memory}};
    size_t i;
    for (i = 0; i < sizeof(tests)/sizeof(tests[0]); ++i)
        if (argc == 1 || strcmp(argv[1], tests[i].name) == 0)
        {
            if (tests[i].run()) return 1;
            printf("PASS %s\n", tests[i].name);
            if (argc > 1) return 0;
        }
    return argc > 1 ? 2 : 0;
}
