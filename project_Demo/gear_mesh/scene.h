#ifndef GEAR_SCENE_H
#define GEAR_SCENE_H
#include "MODEL/GEAR/YMPME_Gear.h"
#include "YMGRE_RenderContext.h"
#include "YMGRE_List.h"
typedef struct gear_part_
{
	GRE_Object4d object;
	gre_vertex4d *local;
	gre_fvector4d *normal;
	int owner, guide;
	pme_vec3 offset;
} gear_part;
typedef struct gear_scene_
{
	pme_gear_pair pair;
	gear_part parts[20];
	int count;
	gre_list objects, lights, materials;
	GRE_Light4d ambient, key;
	GRE_Camera4d camera;
	GRE_RenderWorkspace workspace;
} gear_scene;
int GearScene_Init(gear_scene *s);
int GearScene_InitMode(gear_scene *s, pme_gear_mode mode, int width, int height);
void GearScene_Render(gear_scene *s, int view, int guides, int wire);
void GearScene_Destroy(gear_scene *s);
#endif
