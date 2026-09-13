#include "YMPME_Joint.h"
#include <string.h>

static int sameVec(pme_vec3 a, pme_vec3 b) { return a.x == b.x && a.y == b.y && a.z == b.z; }
static int sameQuat(pme_quaternion a, pme_quaternion b) {
    return a.x == b.x && a.y == b.y && a.z == b.z && a.w == b.w;
}
static pme_vec3 axis(const pme_joint *j, unsigned k) {
    if (j->type == PME_JOINT_PRISMATIC && k == 2)
        return j->rows[4].angularB;
    return j->rows[k].linearB;
}
pme_result YMPME_Joint_CacheInit(pme_joint_cache *c, pme_joint_cache_entry *entries, size_t n) {
    if (!c || (n && !entries))
        return PME_INVALID_ARGUMENT;
    memset(c, 0, sizeof(*c));
    c->entries = entries;
    c->capacity = n;
    c->factor = .75f;
    return PME_OK;
}
void YMPME_Joint_CacheClear(pme_joint_cache *c) {
    if (!c)
        return;
    c->count = c->hits = c->rowHits = c->bodyCount = 0;
    c->interval = 0;
    c->time = 0;
    c->bodies = NULL;
    c->joints = NULL;
}
pme_result YMPME_Joint_ValidateCache(const pme_joint_cache *c, size_t n) {
    if (!c || (c->capacity && !c->entries) || c->count > c->capacity || !isfinite(c->interval) ||
        c->interval < 0 || !isfinite(c->time) || !isfinite(c->factor) || c->factor < 0 ||
        c->factor > 1)
        return PME_INVALID_ARGUMENT;
    for (size_t i = 0; i < c->count; ++i) {
        const pme_joint_cache_entry *e = &c->entries[i];
        if (e->rowCount > 8)
            return PME_INVALID_ARGUMENT;
        for (unsigned k = 0; k < e->rowCount; ++k)
            if (!isfinite(e->impulses[k]))
                return PME_INVALID_ARGUMENT;
        for (unsigned k = 0; k < 3; ++k)
            if (!YMPME_VecFinite(e->axes[k]))
                return PME_INVALID_ARGUMENT;
    }
    return c->capacity < n ? PME_CAPACITY_EXCEEDED : PME_OK;
}
static int compatible(const pme_joint_cache_entry *e, const pme_joint *j) {
    if (!e->enabled || !j->enabled || e->type != j->type || e->bodyA != j->bodyA ||
        e->bodyB != j->bodyB || !sameVec(e->localAnchorA, j->localAnchorA) ||
        !sameVec(e->localAnchorB, j->localAnchorB) || !sameQuat(e->localFrameA, j->localFrameA) ||
        !sameQuat(e->localFrameB, j->localFrameB))
        return 0;
    /* Scalar row impulses belong to their prepared basis. Discard large frame jumps. */
    for (unsigned k = 0; k < 3; ++k)
        if (YMPME_VecDot(e->axes[k], axis(j, k)) < .95f)
            return 0;
    return 1;
}
static void apply(pme_rigid_body *b, pme_vec3 linear, pme_vec3 angular, float impulse) {
    if (!b)
        return;
    b->velocity = YMPME_VecAdd(b->velocity, YMPME_VecScale(linear, impulse));
    b->angularVelocity = YMPME_VecAdd(b->angularVelocity, YMPME_VecScale(angular, impulse));
}
void YMPME_Joint_WarmStart(pme_rigid_body *b, size_t n, pme_joint *j, size_t count,
                           pme_joint_cache *c, float dt, double startTime) {
    c->hits = c->rowHits = 0;
    if (!c->count || c->count != count || c->bodies != b || c->joints != j || c->bodyCount != n ||
        c->interval <= 0 || fabs(c->time - startTime) > fmax(1e-9, (double)dt * 1e-4))
        return;
    float ratio = dt / c->interval;
    if (ratio < .5f || ratio > 2.f)
        return;
    for (size_t i = 0; i < count; ++i) {
        pme_joint *joint = &j[i];
        const pme_joint_cache_entry *e = &c->entries[i];
        if (!compatible(e, joint))
            continue;
        unsigned base = joint->type == PME_JOINT_FIXED ? 6 : 5;
        size_t before = c->rowHits;
        for (unsigned k = 0; k < joint->rowCount && k < e->rowCount; ++k) {
            pme_joint_row *row = &joint->rows[k];
            if (k == base &&
                (e->motorMode != joint->motor.mode || joint->motor.mode == PME_JOINT_MOTOR_OFF))
                continue;
            if (k > base &&
                (!e->limit.enabled || !joint->limit.enabled ||
                 e->limit.lower != joint->limit.lower || e->limit.upper != joint->limit.upper))
                continue;
            if (row->inverseEffectiveMass <= 0)
                continue;
            float impulse = YMPME_Clamp(e->impulses[k] * ratio * c->factor, row->lower, row->upper);
            row->impulse = impulse;
            apply(joint->bodyA == PME_JOINT_WORLD ? NULL : &b[joint->bodyA], row->responseLinearA,
                  row->responseAngularA, impulse);
            apply(&b[joint->bodyB], row->responseLinearB, row->responseAngularB, impulse);
            ++c->rowHits;
        }
        if (c->rowHits != before)
            ++c->hits;
    }
}
void YMPME_Joint_CacheStore(const pme_rigid_body *b, size_t n, const pme_joint *j, size_t count,
                            pme_joint_cache *c, float dt, double time) {
    for (size_t i = 0; i < count; ++i) {
        const pme_joint *joint = &j[i];
        pme_joint_cache_entry *e = &c->entries[i];
        memset(e, 0, sizeof(*e));
        e->type = joint->type;
        e->bodyA = joint->bodyA;
        e->bodyB = joint->bodyB;
        e->enabled = joint->enabled;
        e->localAnchorA = joint->localAnchorA;
        e->localAnchorB = joint->localAnchorB;
        e->localFrameA = joint->localFrameA;
        e->localFrameB = joint->localFrameB;
        e->limit = joint->limit;
        e->motorMode = joint->motor.mode;
        e->rowCount = joint->rowCount;
        if (joint->rowCount)
            for (unsigned k = 0; k < 3; ++k)
                e->axes[k] = axis(joint, k);
        for (unsigned k = 0; k < joint->rowCount; ++k)
            e->impulses[k] = joint->rows[k].impulse;
    }
    c->count = count;
    c->interval = dt;
    c->time = time;
    c->bodies = b;
    c->joints = j;
    c->bodyCount = n;
}
pme_result YMPME_Joint_StepCached(pme_rigid_body *b, size_t n, pme_joint *j, size_t count,
                                  pme_vec3 gravity, float dt, unsigned iterations, double time,
                                  pme_joint_cache *c) {
    if (!c)
        return YMPME_Joint_Step(b, n, j, count, gravity, dt, iterations, time);
    pme_result result = YMPME_Joint_ValidateStep(b, n, j, count, gravity, dt, iterations, time);
    if (result != PME_OK)
        return result;
    result = YMPME_Joint_ValidateCache(c, count);
    if (result != PME_OK)
        return result;
    for (size_t i = 0; i < n; ++i)
        YMPME_Rigid_Begin(&b[i], gravity, dt);
    YMPME_Joint_Prepare(b, j, count, dt);
    YMPME_Joint_WarmStart(b, n, j, count, c, dt, time - dt);
    for (unsigned k = 0; k < iterations; ++k)
        YMPME_Joint_SolveIteration(b, j, count);
    YMPME_Joint_CacheStore(b, n, j, count, c, dt, time);
    for (size_t i = 0; i < n; ++i)
        YMPME_Rigid_End(&b[i], dt, time);
    YMPME_Joint_Finish(b, j, count, dt, time);
    return PME_OK;
}
