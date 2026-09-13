#ifndef CCD_SCENE_H
#define CCD_SCENE_H
#include "YMGRE_List.h"
#include "YMGRE_RenderContext.h"
#include "ccd_lab.h"
typedef struct ccd_scene_
{
    GRE_Object4d balls[2];
    gre_fvector4d *local[2];
    gre_list objects, lights, materials;
    GRE_Light4d ambient, key;
    GRE_Camera4d camera;
    GRE_RenderWorkspace workspace;
} ccd_scene;
int CCDScene_Init(ccd_scene *s);
void CCDScene_Render(ccd_scene *s, const ccd_lab *state, int wire);
void CCDScene_Destroy(ccd_scene *s);
#endif
