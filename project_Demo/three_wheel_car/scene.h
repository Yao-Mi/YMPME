#ifndef PME_CAR_SCENE_H
#define PME_CAR_SCENE_H
#include "YMPME.h"
#include "MODEL/DRIVE/YMPME_Drive.h"
#include "YMGRE_RenderContext.h"
#include "YMGRE_List.h"
#define CAR_MESH_MAX 96
#define CAR_TARGET_COUNT 5
typedef struct car_mesh_
{
	GRE_Object4d object;
	gre_vertex4d* local;
	gre_fvector4d* normals;
	pme_vec3 offset;
	int moving, wheel, caster;
	pme_convex_mesh collisionMesh;
	pme_vec3* collisionVertices;
	uint32_t* collisionIndices;
	pme_vec3* faceAxes;
	pme_vec3* edgeAxes;
	PME_Body collider;
} car_mesh;
typedef struct car_scene_
{
	pme_drive car;
	pme_world world;
	pme_body bodies[64];
	PME_Body proxy;
	car_mesh meshes[CAR_MESH_MAX];
	int meshCount, targets, hits, nearObstacle;
	GRE_Object4d markers[CAR_TARGET_COUNT];
	gre_list objects, lights, materials;
	GRE_Light4d ambient, keyLight;
	GRE_Camera4d camera;
	GRE_RenderWorkspace workspace;
	float elapsed;
} car_scene;
int CarScene_Init(car_scene* s);
void CarScene_Reset(car_scene* s);
void CarScene_SetMeshPrecision(car_scene* s,int enabled);
int CarScene_Step(car_scene* s, float left, float right, int brake, float dt);
void CarScene_Render(car_scene* s, int cameraMode);
void CarScene_Destroy(car_scene* s);
#endif
