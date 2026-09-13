#include "scene.h"
#include "YMGRE_BasicMesh_Gener.h"
#include "YMGRE_Rendering_Pipeline.h"
#include "YMGRE_Camera.h"
#include "YMGRE_Creat.h"
#include "YMGRE_Free.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
static void keepObject(void* data)
{
	(void)data;
}
int BallScene_Init(ball_scene* s)
{
	pme_body_desc d = YMPME_Body_DefaultDesc();
	gre_fvector4d eye = {95, 80, -180, 1}, target = {0, 30, 0, 1};
	memset(s, 0, sizeof(*s));
	if (YMPME_World_Init(&s->world, s->storage, 2, NULL) != PME_OK) return 0;
	d.type = PME_STATIC;
	d.shape = PME_BOX;
	d.halfExtents = YMPME_Vec3(3, 0.25f, 3);
	d.position.y = -0.25f;
	d.restitution = 0.75f;
	s->groundBody = YMPME_World_AddBody(&s->world, &d);
	d = YMPME_Body_DefaultDesc();
	d.position.y = 4;
	d.restitution = 0.75f;
	s->ballBody = YMPME_World_AddBody(&s->world, &d);
	s->ground = YMGRE_MeshGener_Box(120, 10, 120, (GRErgb24){100, 150, 180}, "ground", "");
	s->ball = YMGRE_MeshGener_Sphere(10, 12, 24, (GRErgb24){250, 165, 60}, "ball", "");
	if (!s->groundBody || !s->ballBody || !s->ground || !s->ball) return 0;
	s->groundLocal = malloc((size_t)s->ground->pointNum*sizeof(*s->groundLocal));
	s->ballLocal = malloc((size_t)s->ball->pointNum*sizeof(*s->ballLocal));
	if (!s->groundLocal || !s->ballLocal) return 0;
	memcpy(s->groundLocal, s->ground->pointList, (size_t)s->ground->pointNum*sizeof(*s->groundLocal));
	memcpy(s->ballLocal, s->ball->pointList, (size_t)s->ball->pointNum*sizeof(*s->ballLocal));
	YMGRE_List_Append(&s->objects, sizeof(*s->ground), s->ground);
	YMGRE_List_Append(&s->objects, sizeof(*s->ball), s->ball);
	s->ambient = YMGRE_Creat_Light(0, GRE_GlobalLight, (GRErgb24){255, 255, 255}, 0.3f);
	s->keyLight = YMGRE_Creat_Light(1, GRE_PointLight, (GRErgb24){255, 245, 225}, 0.85f);
	if (!s->ambient || !s->keyLight) return 0;
	s->keyLight->pos = (gre_fvector4d){-70, 130, -90, 1};
	s->keyLight->proper.kc0 = 1;
	s->keyLight->proper.kc1 = s->keyLight->proper.kc2 = 0;
	YMGRE_List_Append(&s->lights, sizeof(*s->ambient), s->ambient);
	YMGRE_List_Append(&s->lights, sizeof(*s->keyLight), s->keyLight);
	s->camera = YMGRE_Creat_Camera(0, 640, 420, 38, 38, 28, 28);
	s->workspace = YMGRE_Creat_RenderWorkspace();
	if (!s->camera || !s->workspace) return 0;
	YMGRE_Camera_Frustum_Init(s->camera, 1, 1000);
	YMGRE_UVNCamera_PositionInit(s->camera, &eye, &target, NULL, 0);
	s->camera->wireFrame = GRE_Render_Solid;
	return 1;
}
void BallScene_Reset(ball_scene* s)
{
	YMPME_Body_SetPosition(s->ballBody, YMPME_Vec3(0, 4, 0));
	s->ballBody->state.velocity = YMPME_Vec3(0, 0, 0);
	s->ballBody->force = YMPME_Vec3(0, 0, 0);
	s->world.accumulator = s->world.droppedTime = 0;
	s->world.stepCount = 0;
}
int BallScene_Render(ball_scene* s, float alpha)
{
	if (YMPME_GRE_SyncObject(s->groundBody, s->ground, s->groundLocal,
		(size_t)s->ground->pointNum, 20, 1) != PME_OK ||
		YMPME_GRE_SyncObject(s->ballBody, s->ball, s->ballLocal,
		(size_t)s->ball->pointNum, 20, alpha) != PME_OK) return 0;
	YMGRE_Camera_TanglePipline_RenderingWithWorkspace(s->camera,
		&s->lights, &s->objects, &s->materials, s->workspace);
	return 1;
}
int BallScene_SaveFrame(ball_scene* s, unsigned step)
{
	char path[64];
	FILE* file;
	size_t i;
	GRE_RenderTarget target = YMGRE_Camera_GetRenderTarget(s->camera);
	snprintf(path, sizeof(path), "ympme_ball_%03u.ppm", step);
	file = fopen(path, "wb");
	if (!file) return 0;
	fprintf(file, "P6\n%u %u\n255\n", target->width, target->height);
	for (i = 0; i < (size_t)target->width*target->height; ++i)
	{
		GRErgb24 c = GRE_FramePixel_To_RGB24(target->data[i]);
		unsigned char pixel[3] = {c.R, c.G, c.B};
		if (fwrite(pixel, 1, 3, file) != 3) { fclose(file); return 0; }
	}
	if (fclose(file) != 0) return 0;
	printf("rendered %s\n", path);
	return 1;
}
void BallScene_Destroy(ball_scene* s)
{
	free(s->groundLocal);
	free(s->ballLocal);
	if (s->workspace) YMGRE_Free_RenderWorkspace(s->workspace);
	if (s->camera) YMGRE_Free_Camera(s->camera);
	YMGRE_List_Clear(&s->lights, keepObject);
	YMGRE_List_Clear(&s->objects, keepObject);
	if (s->ambient) YMGRE_Free_Light(s->ambient);
	if (s->keyLight) YMGRE_Free_Light(s->keyLight);
	if (s->ground) YMGRE_Free_Object(s->ground);
	if (s->ball) YMGRE_Free_Object(s->ball);
	if (s->world.bodies) YMPME_World_Destroy(&s->world);
	memset(s, 0, sizeof(*s));
}
