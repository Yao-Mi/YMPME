#ifndef PME_HAND_BALL_H
#define PME_HAND_BALL_H
#include "rig.h"
enum
{
	BALL_FACES = 128,
	BALL_POINTS = BALL_FACES * 3
};
typedef struct hand_ball_scene_
{
	hand_skin_rig skin;
	pme_rigid_body ball;
	hand_uv_map autoUv;
	float autoCorners[HAND_FACES * 6];
	int autoCharts, sourceUv, arrows;
	size_t arrowCount, rgbClipped;
	int colorMode; /* 0 RGB分量，1 |F|，2 法向压力。 */
	GRErgb24 rgbPatch[HAND_FACES], rgbVertex[HAND_POINTS];
	pme_vec3 wristTarget;
	float tilt, grip, radius, floorZ, floorForce, totalNormal;
	int wire, motion, gravity, lost;
	double motionTime;
	GRE_Object4d object;
	GRE_Material material;
	gre_listnode objectNode, materialNode;
	pme_vec3 local[BALL_POINTS], normals[BALL_FACES];
} hand_ball_scene;
int HandBall_Init(hand_ball_scene *s);
int HandBall_Reset(hand_ball_scene *s);
int HandBall_Step(hand_ball_scene *s);
void HandBall_Render(hand_ball_scene *s);
void HandBall_UpdateRGB(hand_ball_scene *s);
void HandBall_RGBLegend(hand_ball_scene *s);
void HandBall_RenderForces(hand_ball_scene *s);
int HandBall_LocalForce(const hand_ball_scene *s, size_t patch, pme_vec3 *out);
void HandBall_Destroy(hand_ball_scene *s);
const hand_uv_map *HandBall_UV(const hand_ball_scene *s);
void HandBall_Nudge(hand_ball_scene *s, pme_vec3 impulse);
#endif
