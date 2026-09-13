#include "YMPME_CCD.h"
#include <float.h>
#include <string.h>
#define V YMPME_Vec3
#define add YMPME_VecAdd
#define sub YMPME_VecSub
#define mul YMPME_VecScale
#define dot YMPME_VecDot
#define cross YMPME_VecCross
static double dd(pme_vec3 a, pme_vec3 b)
{
    return (double)a.x * b.x + (double)a.y * b.y + (double)a.z * b.z;
}
static float component(pme_vec3 a, int k) { return k == 0 ? a.x : k == 1 ? a.y : a.z; }
static pme_vec3 unit(pme_vec3 a)
{
    double n = sqrt(dd(a, a));
    return n > 1e-20 ? mul(a, (float)(1 / n)) : V(0, 0, 0);
}
pme_result YMPME_Dynamics_CCDInit(pme_ccd *c, const uint8_t *enabled, size_t n,
                                  pme_ccd_event *events, size_t capacity)
{
    if (!c || (n && !enabled) || (capacity && !events))
        return PME_INVALID_ARGUMENT;
    memset(c, 0, sizeof *c);
    c->enabled = enabled;
    c->bodyCount = n;
    c->events = events;
    c->capacity = capacity;
    c->maxImpacts = 8;
    return PME_OK;
}
pme_result PME_CCD_Validate(const pme_dynamics *w)
{
    const pme_ccd *c = w->ccd;
    if (c->bodyCount != w->bodyCount || (c->bodyCount && !c->enabled) ||
        (c->capacity && !c->events) || !c->maxImpacts || c->maxImpacts > 32)
        return PME_INVALID_ARGUMENT;
    size_t active = 0;
    for (size_t i = 0; i < c->bodyCount; ++i)
    {
        if (c->enabled[i] > 1)
            return PME_INVALID_ARGUMENT;
        if (!c->enabled[i])
            continue;
        if (!w->bodies[i].inverseMass)
            return PME_INVALID_ARGUMENT;
        ++active;
        size_t shapes = 0;
        for (size_t k = 0; k < w->colliderCount; ++k)
            if (w->colliders[k].body == i)
            {
                const pme_collider *s = &w->colliders[k];
                ++shapes;
                if (s->type != PME_COLLIDER_SPHERE || s->localPosition.x != 0 ||
                    s->localPosition.y != 0 || s->localPosition.z != 0)
                    return PME_INVALID_ARGUMENT;
            }
        if (shapes != 1)
            return PME_INVALID_ARGUMENT;
        for (size_t k = 0; k < w->jointCount; ++k)
            if (w->joints[k].enabled && (w->joints[k].bodyA == i || w->joints[k].bodyB == i))
                return PME_INVALID_ARGUMENT;
    }
    if (active > SIZE_MAX / c->maxImpacts || c->capacity < active * c->maxImpacts)
        return PME_CAPACITY_EXCEEDED;
    return PME_OK;
}
typedef struct hit_
{
    float t;
    pme_vec3 normal;
    int found;
    size_t collider;
} hit;
/* Candidate normals always point toward the moving sphere. Only entering roots count. */
static void offer(hit *h, double t, pme_vec3 n, pme_vec3 travel)
{
    if (t < 0 || t > 1 || t > h->t || dd(n, travel) >= -1e-10)
        return;
    n = unit(n);
    if (dd(n, n) < .5)
        return;
    h->t = (float)t;
    h->normal = n;
    h->found = 1;
}
static void sphere(hit *h, pme_vec3 p, pme_vec3 d, pme_vec3 center, float radius)
{
    pme_vec3 m = sub(p, center);
    double a = dd(d, d), b = dd(m, d), c = dd(m, m) - (double)radius * radius;
    if (a < 1e-24 || b >= 0)
        return;
    double disc = b * b - a * c;
    if (disc < 0)
        return;
    double t = c <= 0 ? 0 : c / (-b + sqrt(disc)); /* stable near the surface */
    offer(h, t, sub(add(p, mul(d, (float)t)), center), d);
}
static void edge(hit *h, pme_vec3 p, pme_vec3 d, pme_vec3 a, pme_vec3 b, float radius)
{
    pme_vec3 e = sub(b, a), m = sub(p, a);
    double ee = dd(e, e);
    if (ee < 1e-24)
        return;
    double md = dd(m, e) / ee, vd = dd(d, e) / ee;
    pme_vec3 mp = sub(m, mul(e, (float)md)), dp = sub(d, mul(e, (float)vd));
    double aa = dd(dp, dp), bb = dd(mp, dp), cc = dd(mp, mp) - (double)radius * radius;
    if (aa < 1e-24 || bb >= 0)
        return;
    double disc = bb * bb - aa * cc;
    if (disc < 0)
        return;
    double t = cc <= 0 ? 0 : cc / (-bb + sqrt(disc)), u = md + t * vd;
    if (u >= 0 && u <= 1)
        offer(h, t, sub(add(p, mul(d, (float)t)), add(a, mul(e, (float)u))), d);
}
static void triangle(hit *h, pme_vec3 p, pme_vec3 d, float radius, pme_vec3 a, pme_vec3 b,
                     pme_vec3 c)
{
    pme_vec3 n = unit(cross(sub(b, a), sub(c, a)));
    double speed = dd(d, n), dist = dd(sub(p, a), n);
    if (fabs(speed) > 1e-20)
    {
        /* Both sides make triangle winding irrelevant; closed convex meshes remain required. */
        for (int side = -1; side <= 1; side += 2)
        {
            double t = (side * radius - dist) / speed;
            if (t < 0 || t > h->t || t > 1)
                continue;
            pme_vec3 normal = mul(n, (float)side), center = add(p, mul(d, (float)t));
            pme_vec3 q = sub(center, mul(normal, radius));
            pme_vec3 closest = YMPME_Triangle_ClosestPoint(q, a, b, c), delta = sub(q, closest);
            if (dd(delta, delta) < 1e-12)
                offer(h, t, normal, d);
        }
    }
    sphere(h, p, d, a, radius);
    sphere(h, p, d, b, radius);
    sphere(h, p, d, c, radius);
    edge(h, p, d, a, b, radius);
    edge(h, p, d, b, c, radius);
    edge(h, p, d, c, a, radius);
}
static int bounds(pme_vec3 p, pme_vec3 d, pme_vec3 lo, pme_vec3 hi, float radius)
{
    double enter = 0, leave = 1;
    for (int k = 0; k < 3; ++k)
    {
        double x = component(p, k), v = component(d, k), a = component(lo, k) - radius,
               b = component(hi, k) + radius;
        if (fabs(v) < 1e-20)
        {
            if (x < a || x > b)
                return 0;
            continue;
        }
        double t0 = (a - x) / v, t1 = (b - x) / v;
        if (t0 > t1)
        {
            double t = t0;
            t0 = t1;
            t1 = t;
        }
        enter = fmax(enter, t0);
        leave = fmin(leave, t1);
        if (enter > leave)
            return 0;
    }
    return 1;
}
static hit sweep(const pme_collider *c, const pme_rigid_body *body, pme_vec3 p, pme_vec3 d,
                 float radius)
{
    pme_quaternion q = c->localOrientation;
    if (q.x == 0 && q.y == 0 && q.z == 0 && q.w == 0)
        q = YMPME_Quaternion_Identity();
    pme_rotation br = YMPME_Quaternion_Rotation(body->orientation);
    pme_vec3 origin = add(body->position, YMPME_Rotation_Apply(br, c->localPosition));
    pme_rotation r = YMPME_Quaternion_Rotation(YMPME_Quaternion_Multiply(body->orientation, q));
    p = YMPME_Rotation_InverseApply(r, sub(p, origin));
    d = YMPME_Rotation_InverseApply(r, d);
    hit h = {1, V(0, 0, 0), 0, 0};
    if (c->type == PME_COLLIDER_PLANE)
    {
        if (d.y < 0)
            offer(&h, p.y <= radius ? 0 : ((double)radius - p.y) / d.y, V(0, 1, 0), d);
    }
    else if (c->type == PME_COLLIDER_SPHERE)
        sphere(&h, p, d, V(0, 0, 0), radius + c->radius);
    else if (c->type == PME_COLLIDER_BOX)
    {
        pme_vec3 lo = mul(c->halfSize, -1), hi = c->halfSize;
        if (!bounds(p, d, lo, hi, radius))
            return h;
        pme_vec3 v[8];
        for (int i = 0; i < 8; ++i)
            v[i] = V((i & 1 ? 1 : -1) * hi.x, (i & 2 ? 1 : -1) * hi.y, (i & 4 ? 1 : -1) * hi.z);
        static const unsigned idx[36] = {0, 2, 3, 0, 3, 1, 4, 5, 7, 4, 7, 6, 0, 1, 5, 0, 5, 4,
                                         2, 6, 7, 2, 7, 3, 0, 4, 6, 0, 6, 2, 1, 3, 7, 1, 7, 5};
        for (int k = 0; k < 36; k += 3)
            triangle(&h, p, d, radius, v[idx[k]], v[idx[k + 1]], v[idx[k + 2]]);
    }
    else
    {
        const pme_convex_mesh *m = c->mesh;
        if (!bounds(p, d, m->bounds.min, m->bounds.max, radius))
            return h;
        for (size_t k = 0; k < m->triangleCount; ++k)
            triangle(&h, p, d, radius, YMPME_ConvexMesh_Vertex(m,YMPME_ConvexMesh_Index(m,3 * k)),
                     YMPME_ConvexMesh_Vertex(m,YMPME_ConvexMesh_Index(m,3 * k + 1)), YMPME_ConvexMesh_Vertex(m,YMPME_ConvexMesh_Index(m,3 * k + 2)));
    }
    if (h.found)
        h.normal = unit(YMPME_Rotation_Apply(r, h.normal));
    return h;
}
static int filtered(const pme_collider *a, const pme_collider *b)
{
    if (a->group && a->group == b->group)
        return a->group < 0;
    return !(a->category & b->mask) || !(b->category & a->mask);
}
static hit earliest(const pme_dynamics *w, size_t ball, pme_vec3 p, pme_vec3 travel)
{
    hit best = {1, V(0, 0, 0), 0, 0};
    const pme_collider *s = &w->colliders[ball];
    for (size_t k = 0; k < w->colliderCount; ++k)
    {
        const pme_collider *c = &w->colliders[k];
        if (w->bodies[c->body].inverseMass || filtered(s, c))
            continue;
        hit h = sweep(c, &w->bodies[c->body], p, travel, s->radius);
        if (h.found && (!best.found || h.t < best.t))
        {
            best = h;
            best.collider = k;
        }
    }
    return best;
}
static pme_quaternion rotate(pme_quaternion q, pme_vec3 omega, float dt)
{
    float speed = (float)sqrt(dd(omega, omega));
    pme_quaternion dq;
    if (speed < 1e-12f)
        return q;
    if (YMPME_Quaternion_AxisAngle(mul(omega, 1 / speed), speed * dt, &dq) != PME_OK)
        return q;
    q = YMPME_Quaternion_Multiply(dq, q);
    float inv = 1 / sqrtf(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    return (pme_quaternion){q.x * inv, q.y * inv, q.z * inv, q.w * inv};
}
static void impact(pme_dynamics *w, pme_rigid_body *s, size_t ball, hit h, float elapsed, float dt)
{
    const pme_collider *a = &w->colliders[h.collider], *b = &w->colliders[ball];
    pme_ccd_event *e = &w->ccd->events[w->ccd->count++];
    memset(e, 0, sizeof *e);
    e->colliderA = h.collider;
    e->colliderB = ball;
    e->bodyA = a->body;
    e->bodyB = b->body;
    e->time = elapsed;
    e->normal = h.normal;
    e->originA = w->bodies[a->body].position;
    e->originB = s->position;
    pme_vec3 arm = mul(h.normal, -b->radius);
    e->point = add(s->position, arm);
    float vn = dot(s->velocity, h.normal),
          restitution = vn < -w->bounceThreshold ? fminf(a->restitution, b->restitution) : 0;
    e->normalImpulse = fmaxf(0, -(1 + restitution) * vn / s->inverseMass);
    YMPME_Rigid_Impulse(s, mul(h.normal, e->normalImpulse), V(0, 0, 0));
    /* Solve a 2D Coulomb disk at the sphere surface, including spin/inertia. */
    pme_vec3 u = unit(cross(h.normal, fabsf(h.normal.x) < .7f ? V(1, 0, 0) : V(0, 1, 0))),
             v = cross(h.normal, u);
    pme_vec3 au = cross(arm, u), av = cross(arm, v), iu = YMPME_Rigid_InverseInertia(s, au),
             iv = YMPME_Rigid_InverseInertia(s, av);
    double ku = s->inverseMass + dd(au, iu), kv = s->inverseMass + dd(av, iv), k = dd(au, iv),
           den = ku * kv - k * k;
    pme_vec3 speed = add(s->velocity, cross(s->angularVelocity, arm));
    double su = dd(speed, u), sv = dd(speed, v), x = 0, y = 0;
    if (den > 1e-20)
    {
        x = (-su * kv + k * sv) / den;
        y = (-sv * ku + k * su) / den;
    }
    double mag = hypot(x, y), limit = sqrt((double)a->friction * b->friction) * e->normalImpulse;
    if (mag > limit && mag > 0)
    {
        x *= limit / mag;
        y *= limit / mag;
    }
    e->tangentImpulse = add(mul(u, (float)x), mul(v, (float)y));
    YMPME_Rigid_Impulse(s, e->tangentImpulse, cross(arm, e->tangentImpulse));
    pme_vec3 impulse = add(mul(h.normal, e->normalImpulse), e->tangentImpulse);
    e->reactionB.force = mul(impulse, 1 / dt);
    e->reactionB.torque = mul(cross(arm, impulse), 1 / dt);
    e->reactionA.force = mul(e->reactionB.force, -1);
    e->reactionA.torque = cross(sub(e->point, e->originA), e->reactionA.force);
}
void PME_CCD_Integrate(pme_dynamics *w, size_t body, float dt, double time)
{
    pme_rigid_body *b = &w->bodies[body], state = *b;
    size_t ball = 0;
    while (w->colliders[ball].body != body)
        ++ball;
    float elapsed = 0, remaining = dt;
    unsigned impacts = 0;
    while (remaining > 0)
    {
        pme_vec3 travel = mul(state.velocity, remaining);
        hit h = earliest(w, ball, state.position, travel);
        if (!h.found)
        {
            state.position = add(state.position, travel);
            state.orientation = rotate(state.orientation, state.angularVelocity, remaining);
            break;
        }
        if (impacts == w->ccd->maxImpacts)
        {
            ++w->ccd->clampedBodies;
            break;
        }
        float advance = remaining * h.t;
        state.position = add(state.position, mul(state.velocity, advance));
        state.orientation = rotate(state.orientation, state.angularVelocity, advance);
        elapsed += advance;
        remaining -= advance;
        impact(w, &state, ball, h, elapsed, dt);
        ++impacts;
        /* Small outward skin prevents repeated zero-time roots from round-off. */
        state.position =
            add(state.position, mul(h.normal, fmaxf(1e-7f, w->colliders[ball].radius * 1e-5f)));
    }
    b->velocity = state.velocity;
    b->angularVelocity = state.angularVelocity;
    YMPME_Rigid_End(b, dt, time); /* acceleration/load feedback committed once */
    b->position = state.position;
    b->orientation = state.orientation;
    b->feedback.position = b->position;
    b->feedback.rotation = YMPME_Quaternion_Rotation(b->orientation);
}
