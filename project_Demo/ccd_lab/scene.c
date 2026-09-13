#include "scene.h"
#include "YMGRE_BasicMesh_Gener.h"
#include "YMGRE_Camera.h"
#include "YMGRE_Creat.h"
#include "YMGRE_Free.h"
#include "YMGRE_Rendering_Pipeline.h"
#include <string.h>
#include <stdlib.h>
#define V YMPME_Vec3
static gre_fvector4d point(pme_vec3 p, int lane)
{
    return (gre_fvector4d){p.x * 100, (p.y + (lane ? -.72f : .72f)) * 100, p.z * 100, 1};
}
int CCDScene_Init(ccd_scene *s)
{
    memset(s, 0, sizeof *s);
    for (int i = 0; i < 2; ++i)
    {
        s->balls[i] = YMGRE_MeshGener_Sphere(
            10, 12, 20, i ? (GRErgb24){88, 229, 175} : (GRErgb24){247, 143, 104}, "ball", "");
        if (!s->balls[i])
            return 0;
        s->local[i] = malloc((size_t)s->balls[i]->pointNum * sizeof(*s->local[i]));
        if (!s->local[i]) return 0;
        for (int k=0;k<s->balls[i]->pointNum;++k) s->local[i][k]=s->balls[i]->pointList[k].pos;
        YMGRE_List_Append(&s->objects, sizeof(*s->balls[i]), s->balls[i]);
    }
    s->ambient = YMGRE_Creat_Light(0, GRE_GlobalLight, (GRErgb24){255, 255, 255}, .65f);
    s->key = YMGRE_Creat_Light(1, GRE_PointLight, (GRErgb24){255, 244, 231}, .8f);
    if (!s->ambient || !s->key)
        return 0;
    s->key->pos = (gre_fvector4d){-180, 250, 350, 1};
    s->key->proper.kc0 = 1;
    s->key->proper.kc1 = s->key->proper.kc2 = 0;
    YMGRE_List_Append(&s->lights, sizeof(*s->ambient), s->ambient);
    YMGRE_List_Append(&s->lights, sizeof(*s->key), s->key);
    s->camera = YMGRE_Creat_Camera(0, 1000, 580, 34, 34, 24, 24);
    s->workspace = YMGRE_Creat_RenderWorkspace();
    if (!s->camera || !s->workspace)
        return 0;
    YMGRE_Camera_Frustum_Init(s->camera, 1, 2000);
    return 1;
}
void CCDScene_Render(ccd_scene *s, const ccd_lab *state, int wire)
{
    for (int i = 0; i < 2; ++i)
    {
        gre_fvector4d center = point(state->lanes[i].bodies[1].position, i);
        s->balls[i]->WorldCoordinate = center;
        for (int k=0;k<s->balls[i]->pointNum;++k) {
            gre_fvector4d p=s->local[i][k];
            s->balls[i]->pointList[k].pos=(gre_fvector4d){p.x+center.x,p.y+center.y,p.z+center.z,1};
        }
        s->balls[i]->wireFrame = (uint8)wire;
    }
    gre_fvector4d eye = {50, 0, 480, 1}, target = {50, 0, 0, 1};
    YMGRE_UVNCamera_PositionInit(s->camera, &eye, &target, NULL, 0);
    YMGRE_Camera_TanglePipline_RenderingWithWorkspace(s->camera, &s->lights, &s->objects,
                                                      &s->materials, s->workspace);
    gre_line3d lines[80];
    unsigned n = 0;
    for (int lane = 0; lane < 2; ++lane)
    {
        const ccd_lane *l = &state->lanes[lane];
        GRErgb24 c = lane ? (GRErgb24){88, 229, 175} : (GRErgb24){247, 143, 104};
        lines[n++] =
            (gre_line3d){point(V(-1, 0, 0), lane), point(l->bodies[1].position, lane), c, 2};
        for (int i = 0; i < 8; ++i)
            for (int bit = 1; bit <= 4; bit *= 2)
                if (!(i & bit))
                    lines[n++] = (gre_line3d){point(l->vertices[i], lane),
                                              point(l->vertices[i | bit], lane),
                                              {150, 177, 210},
                                              2};
        lines[n++] = (gre_line3d){
            point(V(-1, -.12f, 0), lane), point(V(-1, .12f, 0), lane), {154, 173, 190}, 1};
        if (l->hits)
        {
            pme_vec3 p = l->events[0].point,
                     tip = YMPME_VecAdd(p, YMPME_VecScale(l->events[0].normal, .32f));
            lines[n++] = (gre_line3d){point(p, lane), point(tip, lane), {252, 228, 103}, 3};
        }
    }
    YMGRE_Camera_LineList_Rendering(s->camera, lines, n, 0);
}
static void keep(void *p) { (void)p; }
void CCDScene_Destroy(ccd_scene *s)
{
    if (s->workspace)
        YMGRE_Free_RenderWorkspace(s->workspace);
    if (s->camera)
        YMGRE_Free_Camera(s->camera);
    YMGRE_List_Clear(&s->objects, keep);
    YMGRE_List_Clear(&s->lights, keep);
    for (int i = 0; i < 2; ++i)
        {
            free(s->local[i]);
            if (s->balls[i]) YMGRE_Free_Object(s->balls[i]);
        }
    if (s->ambient)
        YMGRE_Free_Light(s->ambient);
    if (s->key)
        YMGRE_Free_Light(s->key);
}
