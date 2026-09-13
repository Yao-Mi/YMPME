#ifndef SOFT_FINGER_RENDER_H
#define SOFT_FINGER_RENDER_H
#include "soft_finger.h"
#include "../grasp_lab/scene.h"
typedef struct soft_view_
{
	grasp_scene scene;
	gre_vertex4d *ballRest;
	int ballPart;
	GRE_Object4d pads[2];
	GRE_RenderTarget indentation[2];
} soft_view;
int SoftView_Init(soft_view *v);
void SoftView_Draw(soft_view *v, const soft_finger *s, int camera, int wire);
void SoftView_Destroy(soft_view *v);
#endif
