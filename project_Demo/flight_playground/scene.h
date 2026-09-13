#ifndef FLIGHT_SCENE_H
#define FLIGHT_SCENE_H
#include "MODEL/AIRCRAFT/YMPME_Aircraft.h"
#include "YMGRE_List.h"
#include "YMGRE_RenderContext.h"
#define FLIGHT_PART_MAX 180
typedef struct flight_part_
{
	GRE_Object4d object;
	gre_vertex4d *local;
	gre_fvector4d *normal;
	pme_vec3 offset;
	int owner, motion;
} flight_part;
typedef struct flight_scene_
{
	pme_aircraft aircraft[2];
	flight_part parts[FLIGHT_PART_MAX];
	int count;
	gre_list objects, lights, materials;
	GRE_Light4d ambient, key;
	GRE_Camera4d camera[2];
	GRE_RenderWorkspace workspace[2];
} flight_scene;
int FlightScene_Init(flight_scene *scene);
int FlightScene_Reset(flight_scene *scene);
void FlightScene_Render(flight_scene *scene, int view);
void FlightScene_Destroy(flight_scene *scene);
#endif
