#ifndef CCD_LAB_H
#define CCD_LAB_H
#include "DYNAMICS/YMPME_Dynamics.h"
#define CCD_LAB_DT .02f
typedef struct ccd_lane_
{
    pme_rigid_body bodies[2];
    pme_collider colliders[2];
    pme_dynamic_contact contacts[16];
    pme_dynamics world;
    pme_ccd ccd;
    uint8_t flags[2];
    pme_ccd_event events[8];
    pme_vec3 vertices[8], faces[12], edges[36];
    pme_convex_mesh mesh;
    float impactTime, impulse;
    unsigned hits;
} ccd_lane;
typedef struct ccd_lab_
{
    ccd_lane lanes[2];
    int shape, bounce;
    unsigned steps;
} ccd_lab;
int CCDLab_Init(ccd_lab *s, int shape, int bounce);
int CCDLab_Step(ccd_lab *s);
#endif
