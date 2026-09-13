#ifndef TRACK_SCENE_H
#define TRACK_SCENE_H
#include "YMGRE_List.h"
#include "YMGRE_RenderContext.h"
#include "track.h"
typedef struct track_part_ {
  GRE_Object4d object;
  gre_vertex4d *local;
  gre_fvector4d *normals;
  GRErgb24 color;
  int body;
} track_part;
typedef struct track_scene_ {
  track_part parts[44];
  int count;
  gre_list objects, lights, materials;
  GRE_Light4d ambient, key;
  GRE_Camera4d camera;
  GRE_RenderWorkspace workspace;
} track_scene;
int TrackScene_Init(track_scene *s, const track_lab *state);
void TrackScene_Render(track_scene *s, const track_lab *state, int view,
                       int wire, int markers);
void TrackScene_Destroy(track_scene *s);
#endif
