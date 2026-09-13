#ifndef FLEET_PLAYGROUND_H
#define FLEET_PLAYGROUND_H
#include "YMPME.h"
#include "MODEL/VEHICLE/YMPME_Vehicle.h"
#include "terrain.h"
#include "YMGRE_RenderContext.h"
#include "YMGRE_List.h"
#define FLEET_PART_MAX 256
typedef enum part_kind_ { PART_FIXED,PART_BODY,PART_WHEEL,PART_ROD,PART_COIL,PART_TREAD,PART_FORK } part_kind;
typedef struct fleet_part_
{
	GRE_Object4d object;
	gre_vertex4d* local;
	gre_fvector4d* normals;
	part_kind kind;
	int owner,support,index;
	pme_vec3 offset;
	pme_convex_mesh mesh;
	pme_vec3* vertices; uint32_t* indices; pme_vec3* faces; pme_vec3* edges;
	PME_Body collider;
} fleet_part;
typedef struct playground_
{
	pme_terrain terrain;
	pme_vehicle vehicle[PME_VEHICLE_COUNT];
	pme_world world; pme_body bodies[64];
	fleet_part part[FLEET_PART_MAX]; int partCount,selected,contacts;
	gre_list objects,lights,materials;
	GRE_Light4d ambient,keyLight;
	GRE_Camera4d camera; GRE_RenderWorkspace workspace;
	float elapsed;
} playground;
int Playground_Init(playground* p,const char* heightmap);
void Playground_Reset(playground* p);
int Playground_Step(playground* p,float throttle,float steer,int brake,float dt);
void Playground_Render(playground* p,int cameraMode,int wireframe);
void Playground_Destroy(playground* p);
#endif
