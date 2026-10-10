#ifndef TRACKED_VEHICLE_H
#define TRACKED_VEHICLE_H
#include "MODEL/SUSPENSION/YMPME_Suspension.h"
#include "YMPME.h"
#define TV_HZ 240
#define TV_DT (1.f / TV_HZ)
#define TV_TRACK_OFFSET .635f
#define TV_STATIC_END 12
#define TV_LINK_FIRST 12
#define TV_STRAIGHT_LINKS 10
#define TV_ARC_LINKS 4
#define TV_LINKS_PER_SIDE (2 * (TV_STRAIGHT_LINKS + TV_ARC_LINKS))
#define TV_LINKS (2 * TV_LINKS_PER_SIDE)
#define TV_BOGIE_FIRST (TV_LINK_FIRST + TV_LINKS)
#define TV_BOGIE_STRIDE 7
#define TV_BODIES (TV_BOGIE_FIRST + 2 * TV_BOGIE_STRIDE)
#define TV_JOINTS (TV_LINKS + 2 * TV_BOGIE_STRIDE)
#define TV_COLLIDERS (TV_BODIES + 20)
#define TV_CONTACTS 1024
#define TV_WHEEL_SEGMENTS 12
#define TV_MESH_VERTICES (2 * TV_WHEEL_SEGMENTS)
#define TV_MESH_TRIANGLES (4 * TV_WHEEL_SEGMENTS - 4)
typedef struct tv_geometry_ {
    pme_convex_mesh mesh;
    pme_vec3 vertices[TV_MESH_VERTICES], faceAxes[TV_MESH_TRIANGLES],
        edgeAxes[TV_MESH_TRIANGLES * 3];
    PME_Index indices[TV_MESH_TRIANGLES * 3];
} tv_geometry;
typedef struct tracked_vehicle_ {
    pme_rigid_body bodies[TV_BODIES];
    pme_joint joints[TV_JOINTS];
    pme_collider colliders[TV_COLLIDERS];
    pme_dynamic_contact contacts[TV_CONTACTS];
    pme_contact_cache_entry entries[TV_CONTACTS];
    pme_contact_cache cache;
    pme_joint_cache_entry jointEntries[TV_JOINTS];
    pme_joint_cache jointCache;
    pme_dynamics world;
    tv_geometry geometry[4];
    pme_suspension_desc suspension;
    float radius, roadRadius, pitch, halfSpan, mass;
    float appliedSpeed[2], motorTorque[2], wheelSpeed[2], beltSpeed[2], traction[2];
    float tension[2], suspensionForce[2], suspensionTravel[2];
    float friction, wheelFriction, support, penetration, anchorError, lateralError;
    pme_wrench wallLoad;
    pme_vec3 terrainLoad;
    int motor, scenario;
    unsigned long steps;
    pme_result status;
} tracked_vehicle;
int TV_Init(tracked_vehicle *s, int scenario);
int TV_Step(tracked_vehicle *s, float throttle, float steer, int brake);
float TV_Yaw(const tracked_vehicle *s);
int TV_LinkSide(size_t body);
#endif
