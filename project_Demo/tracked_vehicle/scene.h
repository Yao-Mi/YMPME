#ifndef TRACKED_SCENE_H
#define TRACKED_SCENE_H
#include "YMGRE_List.h"
#include "YMGRE_RenderContext.h"
#include "vehicle.h"
#define TV_PARTS (TV_COLLIDERS + 2)
typedef struct tv_part_ {
    GRE_Object4d object;
    gre_vertex4d *local;
    gre_fvector4d *normals;
    GRErgb24 color;
    int collider, decoration;
} tv_part;
typedef struct tv_scene_ {
    tv_part parts[TV_PARTS];
    int count;
    gre_list objects, lights, materials;
    GRE_Light4d ambient, key;
    GRE_Camera4d camera;
    GRE_RenderWorkspace workspace;
} tv_scene;
int TVScene_Init(tv_scene *s, const tracked_vehicle *v);
void TVScene_Render(tv_scene *s, const tracked_vehicle *v, int view, int wire, int forces);
void TVScene_Destroy(tv_scene *s);
#endif
