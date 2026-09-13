#ifndef SENSOR_LAB_SCENE_H
#define SENSOR_LAB_SCENE_H
#include "rig.h"
#include "lidar_projection.h"
#include "YMGRE_RenderContext.h"
#include "YMGRE_List.h"
typedef struct sensor_part_
{
	GRE_Object4d object;
	gre_vertex4d *local;
	gre_fvector4d *normal;
	int body;
	pme_vec3 offset;
} sensor_part;
typedef struct sensor_scene_
{
	sensor_rig rig;
	sensor_lidar_projection projection;
	unsigned laserProjection;
	sensor_part parts[14];
	int count;
	GRE_Camera4d camera, radar, depth, skin;
	GRE_RenderWorkspace workspace;
	GRE_Light4d ambient, key;
	gre_list objects, lights, materials;
} sensor_scene;
int SensorScene_Init(sensor_scene *s);
void SensorScene_Render(sensor_scene *s, int view);
int SensorScene_CheckHandedness(sensor_scene *s);
void SensorScene_Destroy(sensor_scene *s);
#endif
