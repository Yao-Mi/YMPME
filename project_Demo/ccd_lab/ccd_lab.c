#include "ccd_lab.h"
#include <string.h>
#define V YMPME_Vec3
static const PME_Index indices[36] = {0, 2, 3, 0, 3, 1, 4, 5, 7, 4, 7, 6, 0, 1, 5, 0, 5, 4,
                                     2, 6, 7, 2, 7, 3, 0, 4, 6, 0, 6, 2, 1, 3, 7, 1, 7, 5};
int CCDLab_Init(ccd_lab *s, int shape, int bounce)
{
    if (!s || shape < 0 || shape > 2)
        return 0;
    memset(s, 0, sizeof *s);
    s->shape = shape;
    s->bounce = bounce;
    for (int lane = 0; lane < 2; ++lane)
    {
        ccd_lane *l = &s->lanes[lane];
        for (int i = 0; i < 2; ++i)
            if (YMPME_Rigid_Init(&l->bodies[i], i ? 1 : 0, V(.004f, .004f, .004f),
                                 V(i ? -1.f : 0, 0, 0), YMPME_Quaternion_Identity()) != PME_OK)
                return 0;
        l->bodies[1].velocity = V(100, 0, 0);
        l->colliders[0] = YMPME_Dynamics_Collider(0, shape == 0   ? PME_COLLIDER_PLANE
                                                     : shape == 1 ? PME_COLLIDER_BOX
                                                                  : PME_COLLIDER_MESH);
        l->colliders[0].halfSize = V(.01f, .5f, .3f);
        if (shape == 0 && YMPME_Quaternion_AxisAngle(V(0, 0, 1), 1.5707963268f,
                                                     &l->colliders[0].localOrientation) != PME_OK)
            return 0;
        for (int k = 0; k < 8; ++k)
            l->vertices[k] =
                V((k & 1 ? 1 : -1) * .01f + (shape == 2 ? (k & 2 ? 1 : -1) * .125f : 0),
                  (k & 2 ? 1 : -1) * .5f, (k & 4 ? 1 : -1) * .3f);
        if (YMPME_ConvexMesh_Init(&l->mesh, l->vertices, 8, indices, 12, l->faces, 12, l->edges,
                                  36) != PME_OK)
            return 0;
        l->colliders[0].mesh = &l->mesh;
        l->colliders[1] = YMPME_Dynamics_Collider(1, PME_COLLIDER_SPHERE);
        l->colliders[1].radius = .1f;
        for (int i = 0; i < 2; ++i)
        {
            l->colliders[i].friction = 0;
            l->colliders[i].restitution = bounce ? 1 : 0;
        }
        l->world = YMPME_Dynamics_Default();
        l->world.gravity = V(0, 0, 0);
        l->world.bodies = l->bodies;
        l->world.bodyCount = 2;
        l->world.colliders = l->colliders;
        l->world.colliderCount = 2;
        l->world.contacts = l->contacts;
        l->world.contactCapacity = 16;
        l->flags[1] = 1;
        if (YMPME_Dynamics_CCDInit(&l->ccd, l->flags, 2, l->events, 8) != PME_OK)
            return 0;
        if (lane)
            l->world.ccd = &l->ccd;
    }
    return 1;
}
int CCDLab_Step(ccd_lab *s)
{
    for (int i = 0; i < 2; ++i)
    {
        ccd_lane *l = &s->lanes[i];
        if (YMPME_Dynamics_Step(&l->world, CCD_LAB_DT, l->world.time + CCD_LAB_DT) != PME_OK)
            return 0;
        for (size_t k = 0; k < l->ccd.count; ++k)
        {
            l->hits++;
            l->impactTime = l->events[k].time;
            l->impulse += l->events[k].normalImpulse;
        }
    }
    ++s->steps;
    return 1;
}
