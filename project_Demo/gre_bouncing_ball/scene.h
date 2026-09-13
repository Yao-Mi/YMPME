/* 游戏场景由应用拥有，窗口与离屏输出共享同一物理/渲染链路。 */
#ifndef YMPME_BALL_SCENE_H
#define YMPME_BALL_SCENE_H
#include "YMPME.h"
#include "BRIDGE/YMPME_GRE.h"
#include "YMGRE_RenderContext.h"
#include "YMGRE_List.h"
typedef struct ball_scene_
{
	pme_world world;
	pme_body storage[2];
	PME_Body groundBody, ballBody;
	GRE_Object4d ground, ball;
	GRE_Camera4d camera;
	GRE_RenderWorkspace workspace;
	GRE_Light4d ambient, keyLight;
	gre_vertex4d* groundLocal;
	gre_vertex4d* ballLocal;
	gre_list objects, lights, materials;
} ball_scene;
int BallScene_Init(ball_scene* scene);
void BallScene_Destroy(ball_scene* scene);
void BallScene_Reset(ball_scene* scene);
int BallScene_Render(ball_scene* scene, float alpha);
int BallScene_SaveFrame(ball_scene* scene, unsigned step);
#ifdef YMPME_GRE_WINDOW
int BallWindow_Run(ball_scene* scene, int smokeTest);
#endif
#endif
