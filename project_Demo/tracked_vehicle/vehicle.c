#include "vehicle.h"
#include <stdlib.h>
#include <string.h>
#define V YMPME_Vec3
#define ADD YMPME_VecAdd
#define SUB YMPME_VecSub
#define SCALE YMPME_VecScale
#define DOT YMPME_VecDot
#define CROSS YMPME_VecCross
#define PI 3.14159265358979323846f
static pme_quaternion axis(pme_vec3 a, float angle) {
    pme_quaternion q;
    YMPME_Quaternion_AxisAngle(a, angle, &q);
    return q;
}
static int body(tracked_vehicle *s, int i, float mass, pme_vec3 half, pme_vec3 p,
                pme_quaternion q) {
    if (YMPME_Rigid_Init(&s->bodies[i], mass, YMPME_Rigid_BoxInertia(mass, SCALE(half, 2)), p, q) !=
        PME_OK)
        return 0;
    s->mass += mass;
    s->colliders[i] = YMPME_Dynamics_Collider((size_t)i, PME_COLLIDER_BOX);
    s->colliders[i].halfSize = half;
    s->colliders[i].category = 32;
    s->colliders[i].mask = 16;
    s->colliders[i].friction = .3f;
    s->colliders[i].restitution = 0;
    return 1;
}
/* Convex cylindrical wheel, axle X. Render and collision share these buffers. */
static int cylinder(tv_geometry *g, float radius, float width) {
    for (int end = 0; end < 2; ++end)
        for (int i = 0; i < TV_WHEEL_SEGMENTS; ++i) {
            float a = 2 * PI * i / TV_WHEEL_SEGMENTS;
            g->vertices[end * TV_WHEEL_SEGMENTS + i] =
                V((end ? 1 : -1) * width / 2, radius * cosf(a), radius * sinf(a));
        }
    size_t k = 0;
    int n = TV_WHEEL_SEGMENTS;
    for (int i = 1; i < n - 1; ++i) {
        g->indices[k++] = 0;
        g->indices[k++] = i + 1;
        g->indices[k++] = i;
        g->indices[k++] = n;
        g->indices[k++] = n + i;
        g->indices[k++] = n + i + 1;
    }
    for (int i = 0; i < n; ++i) {
        int j = (i + 1) % n;
        g->indices[k++] = i;
        g->indices[k++] = j;
        g->indices[k++] = n + j;
        g->indices[k++] = i;
        g->indices[k++] = n + j;
        g->indices[k++] = n + i;
    }
    return YMPME_ConvexMesh_Init(&g->mesh, g->vertices, 2 * n, g->indices, k / 3, g->faceAxes,
                                 TV_MESH_TRIANGLES, g->edgeAxes, TV_MESH_TRIANGLES * 3) == PME_OK;
}
static pme_vec3 world(pme_vec3 origin, pme_rotation r, pme_vec3 local) {
    return ADD(origin, YMPME_Rotation_Apply(r, local));
}
static int joint(tracked_vehicle *s, int j, pme_joint_type type, size_t a, size_t b, pme_vec3 p,
                 pme_quaternion q) {
    return YMPME_Joint_Init(&s->joints[j], type, s->bodies, TV_BODIES, a, b, p, q) == PME_OK;
}
int TV_LinkSide(size_t id) {
    return id >= TV_LINK_FIRST && id < TV_LINK_FIRST + TV_LINKS ? (int)(id - TV_LINK_FIRST) / TV_LINKS_PER_SIDE
                                                                : -1;
}
float TV_Yaw(const tracked_vehicle *s) {
    pme_rotation r = YMPME_Quaternion_Rotation(s->bodies[0].orientation);
    return atan2f(r.axis[2].x, r.axis[2].z);
}
int TV_Init(tracked_vehicle *s, int scenario) {
    if (!s || scenario < 0 || scenario > 3)
        return 0;
    memset(s, 0, sizeof(*s));
    s->scenario = scenario;
    s->motor = 1;
    s->friction = .3f;
    s->wheelFriction = 1.4f;
    /* Equal-size wheels; the track pitch follows the end-wheel contact radius. */
    s->radius = .1224f * 1.2f;
    s->roadRadius = s->radius;
    const float linkHalfThickness = .014f;
    const float halfArcAngle = PI / (2 * TV_ARC_LINKS);
    float ring = (s->radius + linkHalfThickness + .001f) / cosf(halfArcAngle);
    s->pitch = 2 * ring * sinf(halfArcAngle);
    s->halfSpan = TV_STRAIGHT_LINKS * s->pitch / 2;
    pme_quaternion q = axis(V(0, 1, 0), scenario == 1 ? .48f : scenario == 3 ? PI / 2 : 0);
    pme_rotation r = YMPME_Quaternion_Rotation(q);
    pme_vec3 origin = scenario == 3   ? V(2.65f, .69f, .28f)
                      : scenario == 2 ? V(3, .48f, -3)
                                      : V(0, .48f, scenario == 1 ? 2.5f : -3);
    origin.y = ring + .07f + (scenario == 3 ? .21f : 0);
    if (!body(s, 0, 12, V(.48f, .15f, s->halfSpan + .09f), world(origin, r, V(0, .35f, 0)), q))
        return 0;
    if (!body(s, 1, 0, V(1, 1, 1), V(0, 0, 0), YMPME_Quaternion_Identity()))
        return 0;
    s->colliders[1].type = PME_COLLIDER_PLANE;
    if (!body(s, 2, 0, V(8, 1, .18f), V(0, 1, 5), YMPME_Quaternion_Identity()) ||
        !body(s, 3, 0, V(.18f, 1, 8), V(-8, 1, 0), YMPME_Quaternion_Identity()) ||
        !body(s, 4, 0, V(.18f, 1, 8), V(8, 1, 0), YMPME_Quaternion_Identity()))
        return 0;
    for (int i = 5; i < TV_STATIC_END; ++i) {
        float h = .035f * (4 - abs(i - 8));
        if (!body(s, i, 0, V(.62f, h / 2, .23f), V(2.65f, h / 2, -1.1f + (i - 5) * .46f),
                  YMPME_Quaternion_Identity()))
            return 0;
    }
    for (int i = 1; i < TV_STATIC_END; ++i) {
        s->colliders[i].category = 16;
        s->colliders[i].mask = UINT32_MAX;
        s->colliders[i].friction = s->friction;
    }
    if (!cylinder(&s->geometry[0], s->radius, .18f) || !cylinder(&s->geometry[1], s->roadRadius, .18f) ||
        !cylinder(&s->geometry[2], s->radius + .016f, .018f) ||
        !cylinder(&s->geometry[3], s->roadRadius + .016f, .018f))
        return 0;
    size_t extra = TV_BODIES;
    for (int side = 0; side < 2; ++side) {
        int first = TV_LINK_FIRST + side * TV_LINKS_PER_SIDE, frame = TV_BOGIE_FIRST + side * TV_BOGIE_STRIDE;
        int row = TV_LINKS + side * TV_BOGIE_STRIDE;
        float x = side ? TV_TRACK_OFFSET : -TV_TRACK_OFFSET;
        uint32_t linkBit = 1u << side, wheelBit = 4u << side;
        pme_vec3 nodes[TV_LINKS_PER_SIDE];
        const int straight = TV_STRAIGHT_LINKS, arc = TV_ARC_LINKS;
        for (int i = 0; i < straight; ++i)
            nodes[i] = V(x, ring, -s->halfSpan + i * s->pitch);
        for (int i = 0; i < arc; ++i) {
            float a = PI / 2 - i * PI / arc;
            nodes[straight + i] = V(x, ring * sinf(a), s->halfSpan + ring * cosf(a));
        }
        for (int i = 0; i < straight; ++i)
            nodes[straight + arc + i] = V(x, -ring, s->halfSpan - i * s->pitch);
        for (int i = 0; i < arc; ++i) {
            float a = -PI / 2 - i * PI / arc;
            nodes[2 * straight + arc + i] = V(x, ring * sinf(a), -s->halfSpan + ring * cosf(a));
        }
        pme_quaternion hinge = YMPME_Quaternion_Multiply(q, axis(V(0, 1, 0), PI / 2));
        for (int i = 0; i < TV_LINKS_PER_SIDE; ++i) {
            pme_vec3 a = nodes[i], b = nodes[(i + 1) % TV_LINKS_PER_SIDE], d = SUB(b, a);
            pme_quaternion linkQ =
                YMPME_Quaternion_Multiply(q, axis(V(1, 0, 0), atan2f(-d.y, d.z)));
            if (!body(s, first + i, .12f, V(.12f, linkHalfThickness, s->pitch * .47f),
                      world(origin, r, SCALE(ADD(a, b), .5f)), linkQ))
                return 0;
            s->colliders[first + i].category = linkBit;
            s->colliders[first + i].mask = wheelBit | 16 | 32;
            s->colliders[first + i].friction = 1.1f;
        }
        if (!body(s, frame, 1, V(.08f, .07f, s->halfSpan + .07f), world(origin, r, V(x, 0, 0)), q) ||
            !body(s, frame + 3, .5f, V(.07f, .07f, .07f), world(origin, r, V(x, 0, s->halfSpan)),
                  q))
            return 0;
        for (int wheel = 0; wheel < 5; ++wheel) {
            int id = wheel < 2 ? frame + 1 + wheel : frame + 2 + wheel;
            float radius = wheel < 2 ? s->radius : s->roadRadius, mass = wheel < 2 ? 1.5f : .8f;
            pme_vec3 pos = V(x, wheel < 2 ? 0 : s->roadRadius - s->radius,
                             wheel == 0   ? -s->halfSpan
                             : wheel == 1 ? s->halfSpan
                                          : (wheel - 3) * s->halfSpan * .5f);
            if (!body(s, id, mass, V(.09f, radius, radius), world(origin, r, pos), q))
                return 0;
            s->bodies[id].inertia =
                V(mass * radius * radius / 2, mass * (radius * radius / 4 + .18f * .18f / 12),
                  mass * (radius * radius / 4 + .18f * .18f / 12));
            s->bodies[id].inverseInertia =
                V(1 / s->bodies[id].inertia.x, 1 / s->bodies[id].inertia.y,
                  1 / s->bodies[id].inertia.z);
            s->colliders[id].type = PME_COLLIDER_MESH;
            s->colliders[id].mesh = &s->geometry[wheel < 2 ? 0 : 1].mesh;
            s->colliders[id].category = wheelBit;
            s->colliders[id].mask = linkBit | 16;
            s->colliders[id].friction = 1.1f;
            for (int end = 0; end < 2; ++end) {
                pme_collider *c = &s->colliders[extra++];
                *c = YMPME_Dynamics_Collider((size_t)id, PME_COLLIDER_MESH);
                c->mesh = &s->geometry[wheel < 2 ? 2 : 3].mesh;
                c->localPosition = V(end ? .133f : -.133f, 0, 0);
                c->category = wheelBit;
                c->mask = linkBit | 16;
                c->friction = .15f;
            }
        }
        for (int i = 0; i < TV_LINKS_PER_SIDE; ++i)
            if (!joint(s, side * TV_LINKS_PER_SIDE + i, PME_JOINT_REVOLUTE, (size_t)(first + i),
                       (size_t)(first + (i + 1) % TV_LINKS_PER_SIDE), world(origin, r, nodes[(i + 1) % TV_LINKS_PER_SIDE]),
                       hinge))
                return 0;
        pme_quaternion vertical = YMPME_Quaternion_Multiply(q, axis(V(1, 0, 0), -PI / 2));
        if (!joint(s, row, PME_JOINT_PRISMATIC, 0, (size_t)frame, s->bodies[frame].position,
                   vertical) ||
            YMPME_Joint_SetLimit(&s->joints[row], (pme_joint_limit){1, -.03f, .16f}) != PME_OK ||
            !joint(s, row + 1, PME_JOINT_REVOLUTE, (size_t)frame, (size_t)(frame + 1),
                   s->bodies[frame + 1].position, hinge) ||
            !joint(s, row + 2, PME_JOINT_PRISMATIC, (size_t)frame, (size_t)(frame + 3),
                   s->bodies[frame + 3].position, q) ||
            YMPME_Joint_SetLimit(&s->joints[row + 2], (pme_joint_limit){1, -.035f, .075f}) !=
                PME_OK ||
            !joint(s, row + 3, PME_JOINT_REVOLUTE, (size_t)(frame + 3), (size_t)(frame + 2),
                   s->bodies[frame + 2].position, hinge))
            return 0;
        for (int i = 0; i < 3; ++i)
            if (!joint(s, row + 4 + i, PME_JOINT_REVOLUTE, (size_t)frame, (size_t)(frame + 4 + i),
                       s->bodies[frame + 4 + i].position, hinge))
                return 0;
    }
    if (extra != TV_COLLIDERS)
        return 0;
    s->suspension = (pme_suspension_desc){2000, 100, .35f, .16f, 4, 800};
    s->world = YMPME_Dynamics_Default();
    s->world.bodies = s->bodies;
    s->world.bodyCount = TV_BODIES;
    s->world.joints = s->joints;
    s->world.jointCount = TV_JOINTS;
    s->world.colliders = s->colliders;
    s->world.colliderCount = TV_COLLIDERS;
    s->world.contacts = s->contacts;
    s->world.contactCapacity = TV_CONTACTS;
    s->world.iterations = 48;
    s->world.slop = .0005f;
    s->world.maxCorrectionSpeed = .3f;
    if (YMPME_Dynamics_CacheInit(&s->cache, s->entries, TV_CONTACTS) != PME_OK ||
        YMPME_Joint_CacheInit(&s->jointCache, s->jointEntries, TV_JOINTS) != PME_OK)
        return 0;
    s->world.cache = &s->cache;
    s->world.jointCache = &s->jointCache;
    return 1;
}
int TV_Step(tracked_vehicle *s, float throttle, float steer, int brake) {
    if (!s || !isfinite(throttle) || !isfinite(steer))
        return 0;
    pme_rigid_body *chassis = &s->bodies[0];
    pme_rotation r = YMPME_Quaternion_Rotation(chassis->orientation);
    for (int side = 0; side < 2; ++side) {
        int frame = TV_BOGIE_FIRST + side * TV_BOGIE_STRIDE, row = TV_LINKS + side * TV_BOGIE_STRIDE;
        pme_rigid_body *bogie = &s->bodies[frame], *carrier = &s->bodies[frame + 3];
        pme_vec3 up = r.axis[1], fwd = r.axis[2],
                 mount = world(chassis->position, r, V(side ? TV_TRACK_OFFSET : -TV_TRACK_OFFSET, 0, 0));
        pme_vec3 velocity =
            ADD(chassis->velocity, CROSS(chassis->angularVelocity, SUB(mount, chassis->position)));
        float distance = DOT(SUB(mount, bogie->position), up),
              rate = DOT(SUB(bogie->velocity, velocity), up), force = 0;
        if (YMPME_Suspension_Force(&s->suspension, distance, rate, &force) != PME_OK)
            return 0;
        if (YMPME_Rigid_AddForceAt(chassis, SCALE(up, force), bogie->position) != PME_OK ||
            YMPME_Rigid_AddForceAt(bogie, SCALE(up, -force), bogie->position) != PME_OK)
            return 0;
        s->suspensionForce[side] = force;
        s->suspensionTravel[side] = s->suspension.rest - distance;
        pme_vec3 base = world(bogie->position, r, V(0, 0, s->halfSpan));
        velocity = ADD(bogie->velocity, CROSS(bogie->angularVelocity, SUB(base, bogie->position)));
        float travel = DOT(SUB(carrier->position, base), fwd),
              speed = DOT(SUB(carrier->velocity, velocity), fwd);
        s->tension[side] = 160 - 350 * travel - 12 * speed;
        if (YMPME_Rigid_AddForceAt(carrier, SCALE(fwd, s->tension[side]), carrier->position) !=
                PME_OK ||
            YMPME_Rigid_AddForceAt(bogie, SCALE(fwd, -s->tension[side]), carrier->position) !=
                PME_OK)
            return 0;
        /* The only propulsion input is bounded torque on the actual drive-wheel hinge. */
        float target = brake ? 0 : (.67f / s->radius) * YMPME_Clamp(throttle + (side ? -steer : steer), -1, 1);
        if (s->world.time < .1)
            target = 0;
        s->appliedSpeed[side] += YMPME_Clamp(target - s->appliedSpeed[side], -1.8f / s->radius * TV_DT, 1.8f / s->radius * TV_DT);
        if (YMPME_Joint_SetMotor(
                &s->joints[row + 1],
                (pme_joint_motor){s->motor ? PME_JOINT_MOTOR_SPEED : PME_JOINT_MOTOR_OFF,
                                  s->appliedSpeed[side], 0, 60}) != PME_OK)
            return 0;
    }
    for (int i = 1; i < TV_STATIC_END; ++i)
        s->colliders[i].friction = s->friction;
    for (int i = TV_BOGIE_FIRST; i < TV_BODIES; ++i)
        if (s->colliders[i].type == PME_COLLIDER_MESH)
            s->colliders[i].friction = s->wheelFriction;
    for (int i = TV_BODIES; i < TV_COLLIDERS; ++i)
        s->colliders[i].friction = s->wheelFriction > 0 ? .15f : 0;
    s->status = YMPME_Dynamics_Step(&s->world, TV_DT, s->world.time + (double)TV_DT);
    if (s->status != PME_OK)
        return 0;
    memset(&s->wallLoad, 0, sizeof(s->wallLoad));
    s->terrainLoad = V(0, 0, 0);
    s->support = s->penetration = s->anchorError = s->lateralError = 0;
    s->traction[0] = s->traction[1] = s->beltSpeed[0] = s->beltSpeed[1] = 0;
    for (int i = 0; i < TV_JOINTS; ++i)
        s->anchorError = fmaxf(s->anchorError, s->joints[i].feedback.anchorError);
    for (size_t i = 0; i < s->world.contactCount; ++i) {
        const pme_dynamic_contact *c = &s->contacts[i];
        s->penetration = fmaxf(s->penetration, c->penetration);
        size_t fixed = c->bodyA > 0 && c->bodyA < TV_STATIC_END ? c->bodyA : c->bodyB;
        if (fixed == 0 || fixed >= TV_STATIC_END)
            continue;
        size_t moving = fixed == c->bodyA ? c->bodyB : c->bodyA;
        pme_wrench load = moving == c->bodyA ? c->reactionA : c->reactionB;
        if (fixed >= 2 && fixed <= 4) {
            s->wallLoad.force = ADD(s->wallLoad.force, load.force);
            s->wallLoad.torque =
                ADD(s->wallLoad.torque, CROSS(SUB(c->point, chassis->position), load.force));
        } else {
            s->support += load.force.y;
            if (fixed >= 5)
                s->terrainLoad = ADD(s->terrainLoad, load.force);
        }
        int side = TV_LinkSide(moving);
        if (side >= 0)
            s->traction[side] += DOT(load.force, r.axis[2]);
    }
    for (int side = 0; side < 2; ++side) {
        int frame = TV_BOGIE_FIRST + side * TV_BOGIE_STRIDE;
        s->motorTorque[side] = s->joints[TV_LINKS + 1 + side * TV_BOGIE_STRIDE].feedback.motorEffort;
        s->wheelSpeed[side] = s->joints[TV_LINKS + 1 + side * TV_BOGIE_STRIDE].feedback.speed;
        const pme_rigid_body *bogie = &s->bodies[frame];
        pme_rotation rr = YMPME_Quaternion_Rotation(bogie->orientation);
        for (int i = 0; i < TV_LINKS_PER_SIDE; ++i) {
            const pme_rigid_body *link = &s->bodies[TV_LINK_FIRST + TV_LINKS_PER_SIDE * side + i];
            pme_vec3 local = YMPME_Rotation_InverseApply(rr, SUB(link->position, bogie->position));
            s->lateralError = fmaxf(s->lateralError, fabsf(local.x));
            pme_vec3 relative =
                SUB(link->velocity,
                    ADD(bogie->velocity,
                        CROSS(bogie->angularVelocity, SUB(link->position, bogie->position))));
            s->beltSpeed[side] += sqrtf(DOT(relative, relative)) / TV_LINKS_PER_SIDE;
        }
    }
    for (int i = 0; i < TV_BODIES; ++i)
        if (!YMPME_Rigid_IsValid(&s->bodies[i]))
            return 0;
    ++s->steps;
    return 1;
}
