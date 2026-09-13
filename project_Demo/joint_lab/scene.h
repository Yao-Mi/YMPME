#ifndef JOINT_LAB_SCENE_H
#define JOINT_LAB_SCENE_H
#include "control/control.h"
#include "YMGRE_RenderContext.h"
#include "YMGRE_List.h"
typedef struct joint_part_
{
	GRE_Object4d object;
	gre_vertex4d *local;
	gre_fvector4d *normal;
	int owner;
	pme_vec3 offset;
} joint_part;
typedef struct joint_scene_
{
	joint_part parts[18];
	int count, kind;
	gre_list objects, lights, materials;
	GRE_Light4d ambient, key;
	GRE_Camera4d camera;
	GRE_RenderWorkspace workspace;
} joint_scene;
int JointScene_Init(joint_scene *s, int kind);
void JointScene_Render(joint_scene *s, const joint_lab_state *state, int view, int wire);
void JointScene_Destroy(joint_scene *s);
#endif
