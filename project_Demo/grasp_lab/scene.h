#ifndef GRASP_SCENE_H
#define GRASP_SCENE_H
#include "grasp.h"
#include "YMGRE_RenderContext.h"
#include "YMGRE_List.h"
typedef struct grasp_part_
{
	GRE_Object4d object;
	gre_vertex4d *local;
	gre_fvector4d *normals;
	GRErgb24 *colors;
	int body, shape;
	pme_vec3 offset;
} grasp_part;
typedef struct grasp_scene_
{
	grasp_part parts[12];
	int count;
	gre_list objects, lights, materials;
	GRE_Light4d ambient, key;
	GRE_Camera4d camera;
	GRE_RenderWorkspace workspace;
	GRE_RenderTarget maps[2];
} grasp_scene;
int GraspScene_Init(grasp_scene *s);
void GraspScene_Render(grasp_scene *s, const grasp_lab *state, int view, int wire, int pressure);
void GraspScene_Destroy(grasp_scene *s);
#endif
