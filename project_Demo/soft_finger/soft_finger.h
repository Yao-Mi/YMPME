#ifndef SOFT_FINGER_DEMO_H
#define SOFT_FINGER_DEMO_H
#include "../grasp_lab/grasp.h"
#include "SOFTBODY/YMPME_SoftPad.h"
#include "SOFTBODY/YMPME_SoftBall.h"
typedef struct soft_ball_proxy_
{
	pme_convex_mesh mesh;
	pme_vec3 vertices[26], faces[48], edges[144];
	PME_Index indices[144];
} soft_ball_proxy;
typedef struct soft_finger_
{
	grasp_lab grasp;
	pme_soft_pad_desc material;
	pme_surface_patch rest[2][GRASP_CELLS];
	pme_soft_pad_sample samples[2][GRASP_CELLS];
	pme_soft_pad_node nodes[2][GRASP_CELLS];
	pme_soft_pad_result result[2];
	pme_soft_ball ball;
	float ballForce;
	int ballProxy;
	soft_ball_proxy ballGeometry[2];
	int hard, ballFirm;
} soft_finger;
int SoftFinger_Init(soft_finger *s, int shape, int hard);
int SoftFinger_BallStep(soft_finger *s);
int SoftFinger_Step(soft_finger *s);
#endif
