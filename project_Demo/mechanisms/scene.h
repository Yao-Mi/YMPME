#ifndef MECHANISMS_SCENE_H
#define MECHANISMS_SCENE_H
#include "YMGRE_List.h"
#include "YMGRE_RenderContext.h"
#include "model.h"
typedef struct mech_part_
{
	GRE_Object4d object;
	gre_vertex4d *local;
	gre_fvector4d *normal;
	float bound;
} mech_part;
typedef struct mech_scene_
{
	mech_part parts[220];
	int count, used, kind;
	gre_list objects, lights, materials;
	GRE_Light4d ambient, key;
	GRE_Camera4d camera;
	GRE_RenderWorkspace workspace;
} mech_scene;
int MechScene_Init(mech_scene *s, int kind, int width, int height);
int MechScene_Render(mech_scene *s, const mechanisms_model *m, int view, int contacts);
void MechScene_Destroy(mech_scene *s);
#endif
