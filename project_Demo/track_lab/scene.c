#include "scene.h"
#include "YMGRE_BasicMesh_Gener.h"
#include "YMGRE_Camera.h"
#include "YMGRE_Creat.h"
#include "YMGRE_Free.h"
#include "YMGRE_Mem.h"
#include "YMGRE_Rendering_Pipeline.h"
#include <stdlib.h>
#include <string.h>
#define V YMPME_Vec3
#define U 100.f
static gre_fvector4d gre(pme_vec3 p) {
  return (gre_fvector4d){p.x * U, p.y * U, p.z * U, 1};
}
static int add(track_scene *s, GRE_Object4d o, int body, GRErgb24 color) {
  if (!o)
    return 0;
  if (s->count >= 44) {
    YMGRE_Free_Object(o);
    return 0;
  }
  track_part *p = &s->parts[s->count++];
  p->object = o;
  p->body = body;
  p->color = color;
  p->local = malloc((size_t)o->pointNum * sizeof(*p->local));
  p->normals = malloc((size_t)o->polygonNum * sizeof(*p->normals));
  if (!p->local || !p->normals)
    return 0;
  memcpy(p->local, o->pointList, (size_t)o->pointNum * sizeof(*p->local));
  for (int i = 0; i < o->polygonNum; ++i)
    p->normals[i] = o->polygonList[i].pN;
  YMGRE_List_Append(&s->objects, sizeof(*o), o);
  return 1;
}
static GRE_Object4d wheel(const track_lab *s) {
  GRE_Object4d o = YMGRE_MeshGener_Cylinder(
      s->radius * U, .18f * U, 48, (GRErgb24){200, 170, 100}, "wheel", "");
  if (!o)
    return NULL;
  /* GRE cylinders use Y as axle; this bench rotates around Z. */
  for (int i = 0; i < o->pointNum; ++i) {
    gre_fvector4d p = o->pointList[i].pos;
    o->pointList[i].pos = (gre_fvector4d){p.x, -p.z, p.y, 1};
  }
  for (int i = 0; i < o->polygonNum; ++i) {
    gre_fvector4d n = o->polygonList[i].pN;
    o->polygonList[i].pN = (gre_fvector4d){n.x, -n.z, n.y, 0};
  }
  return o;
}
int TrackScene_Init(track_scene *s, const track_lab *state) {
  memset(s, 0, sizeof(*s));
  for (int i = 0; i < 40; ++i) {
    GRErgb24 c = i == 0  ? (GRErgb24){242, 107, 75}
                 : i % 2 ? (GRErgb24){86, 113, 128}
                         : (GRErgb24){161, 185, 196};
    if (!add(s,
             YMGRE_MeshGener_Box(state->pitch * .94f * U, .07f * U, .24f * U, c,
                                 "track link", ""),
             i, c))
      return 0;
  }
  for (int i = 40; i < 42; ++i)
    if (!add(s, wheel(state), i,
             i == 40 ? (GRErgb24){226, 166, 63} : (GRErgb24){63, 174, 185}))
      return 0;
  if (!add(s,
           YMGRE_MeshGener_Box(.15f * U, .15f * U, .32f * U,
                               (GRErgb24){225, 219, 196}, "tension slider", ""),
           42, (GRErgb24){225, 219, 196}) ||
      !add(s,
           YMGRE_MeshGener_RectPlane(5 * U, 3 * U, 10, 6,
                                     (GRErgb24){42, 54, 67}, "bench floor", ""),
           43, (GRErgb24){42, 54, 67}))
    return 0;
  s->ambient =
      YMGRE_Creat_Light(0, GRE_GlobalLight, (GRErgb24){255, 255, 255}, .65f);
  s->key = YMGRE_Creat_Light(1, GRE_PointLight, (GRErgb24){255, 244, 231}, .7f);
  if (!s->ambient || !s->key)
    return 0;
  s->key->pos = (gre_fvector4d){-180, 350, 260, 1};
  s->key->proper.kc0 = 1;
  s->key->proper.kc1 = s->key->proper.kc2 = 0;
  YMGRE_List_Append(&s->lights, sizeof(*s->ambient), s->ambient);
  YMGRE_List_Append(&s->lights, sizeof(*s->key), s->key);
  s->camera = YMGRE_Creat_Camera(0, 1100, 560, 34, 34, 24, 24);
  s->workspace = YMGRE_Creat_RenderWorkspace();
  if (!s->camera || !s->workspace)
    return 0;
  YMGRE_Camera_Frustum_Init(s->camera, 1, 2000);
  return 1;
}
void TrackScene_Render(track_scene *s, const track_lab *state, int view,
                       int wire, int markers) {
  unsigned char loaded[TRACK_BODIES] = {0};
  for (size_t i = 0; i < state->world.contactCount; ++i)
    if (state->contacts[i].normalImpulse > 1e-5f) {
      loaded[state->contacts[i].bodyA] = 1;
      loaded[state->contacts[i].bodyB] = 1;
    }
  for (int i = 0; i < s->count; ++i) {
    track_part *p = &s->parts[i];
    GRE_Object4d o = p->object;
    const pme_rigid_body *b = &state->bodies[p->body];
    pme_rotation r = YMPME_Quaternion_Rotation(b->orientation);
    o->WorldCoordinate = gre(b->position);
    o->wireFrame = (uint8)(wire && p->body != 43);
    for (int k = 0; k < o->pointNum; ++k) {
      pme_vec3 v = YMPME_Rotation_Apply(
          r, V(p->local[k].pos.x, p->local[k].pos.y, p->local[k].pos.z));
      o->pointList[k] = p->local[k];
      o->pointList[k].pos =
          (gre_fvector4d){v.x + b->position.x * U, v.y + b->position.y * U,
                          v.z + b->position.z * U, 1};
    }
    for (int k = 0; k < o->polygonNum; ++k) {
      pme_vec3 n = YMPME_Rotation_Apply(
          r, V(p->normals[k].x, p->normals[k].y, p->normals[k].z));
      o->polygonList[k].pN = (gre_fvector4d){n.x, n.y, n.z, 0};
      GRErgb24 c = p->color;
      if (p->body >= 40 && p->body <= 41 && k % 6 < 2)
        c = (GRErgb24){64, 85, 99};
      if (markers && p->body < 40 && loaded[p->body])
        c = (GRErgb24){183, 218, 129};
      if (p->body == 0)
        c = (GRErgb24){242, 107, 75};
      o->polygonList[k].planeColor = c;
    }
  }
  gre_fvector4d eye = gre(view ? V(2.3f, 2.1f, 4.3f) : V(0, 1.0f, 4.3f)),
                target = gre(V(0, .8f, 0));
  YMGRE_UVNCamera_PositionInit(s->camera, &eye, &target, NULL, 0);
  YMGRE_Camera_TanglePipline_RenderingWithWorkspace(
      s->camera, &s->lights, &s->objects, &s->materials, s->workspace);
  gre_line3d lines[800];
  unsigned count = 0;
  for (int i = -5; i <= 5; ++i)
    lines[count++] = (gre_line3d){gre(V(i * .5f, .003f, -1.5f)),
                                  gre(V(i * .5f, .003f, 1.5f)),
                                  {74, 93, 111},
                                  1};
  for (int i = 40; i <= 41; ++i) {
    pme_vec3 c = state->bodies[i].position;
    c.z = .15f;
    pme_vec3 d = YMPME_Rotation_Apply(
        YMPME_Quaternion_Rotation(state->bodies[i].orientation),
        V(state->radius * .8f, 0, 0));
    lines[count++] =
        (gre_line3d){gre(c), gre(YMPME_VecAdd(c, d)), {253, 240, 204}, 3};
    lines[count++] = (gre_line3d){
        gre(V(c.x, 0, -.2f)), gre(V(c.x, c.y, -.2f)), {128, 144, 156}, 5};
  }
  lines[count++] = (gre_line3d){gre(V(state->halfSpan - .16f, .85f, .18f)),
                                gre(V(state->halfSpan + .18f, .85f, .18f)),
                                {243, 208, 105},
                                3};
  /* Outline both contact guide planes without hiding the mechanism. */
  for (int side = 0; side < 2; ++side) {
    float z = side ? .122f : -.122f;
    pme_vec3 corners[4] = {V(-1.55f, .15f, z), V(1.55f, .15f, z),
                           V(1.55f, 1.42f, z), V(-1.55f, 1.42f, z)};
    for (int k = 0; k < 4; ++k)
      lines[count++] = (gre_line3d){
          gre(corners[k]), gre(corners[(k + 1) % 4]), {77, 117, 134}, 1};
  }
  if (markers) {
    for (int i = 0; i < 40; ++i) {
      pme_vec3 a =
          YMPME_VecAdd(state->bodies[i].position,
                       YMPME_Rotation_Apply(YMPME_Quaternion_Rotation(
                                                state->bodies[i].orientation),
                                            state->joints[i].localAnchorA));
      a.z = .14f;
      lines[count++] = (gre_line3d){gre(YMPME_VecAdd(a, V(-.012f, 0, 0))),
                                    gre(YMPME_VecAdd(a, V(.012f, 0, 0))),
                                    {246, 229, 164},
                                    2};
      lines[count++] = (gre_line3d){gre(YMPME_VecAdd(a, V(0, -.012f, 0))),
                                    gre(YMPME_VecAdd(a, V(0, .012f, 0))),
                                    {246, 229, 164},
                                    2};
    }
    for (size_t i = 0; i < state->world.contactCount; ++i) {
      const pme_dynamic_contact *c = &state->contacts[i];
      if (c->normalImpulse < 1e-5f || c->bodyA >= 44 || c->bodyB >= 44 ||
          count + 1 >= 800)
        continue;
      pme_vec3 a = c->point;
      a.z = .15f;
      pme_vec3 n = YMPME_VecScale(c->normal, c->bodyB < 40 ? 1 : -1);
      lines[count++] = (gre_line3d){
          gre(a),
          gre(YMPME_VecAdd(
              a, YMPME_VecScale(
                     n, fminf(.14f, c->normalImpulse / TRACK_DT * .005f)))),
          {116, 244, 186},
          2};
    }
  }
  YMGRE_Camera_LineList_Rendering(s->camera, lines, count, 0);
}
static void keep(void *p) { (void)p; }
void TrackScene_Destroy(track_scene *s) {
  if (s->workspace)
    YMGRE_Free_RenderWorkspace(s->workspace);
  if (s->camera)
    YMGRE_Free_Camera(s->camera);
  YMGRE_List_Clear(&s->objects, keep);
  YMGRE_List_Clear(&s->lights, keep);
  for (int i = 0; i < s->count; ++i) {
    free(s->parts[i].local);
    free(s->parts[i].normals);
    YMGRE_Free_Object(s->parts[i].object);
  }
  if (s->ambient)
    YMGRE_Free_Light(s->ambient);
  if (s->key)
    YMGRE_Free_Light(s->key);
}
