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
#define ADD YMPME_VecAdd
#define SUB YMPME_VecSub
#define SCALE YMPME_VecScale
#define U 100.f
static gre_fvector4d gre(pme_vec3 p) { return (gre_fvector4d){p.x * U, p.y * U, p.z * U, 1}; }
static int add(tv_scene *s, GRE_Object4d o, int collider, int decoration, GRErgb24 color) {
    if (!o)
        return 0;
    if (s->count >= TV_PARTS) {
        YMGRE_Free_Object(o);
        return 0;
    }
    tv_part *p = &s->parts[s->count++];
    p->object = o;
    p->collider = collider;
    p->decoration = decoration;
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
static GRE_Object4d mesh(const pme_convex_mesh *m) {
    GRE_Object4d o =
        YMGRE_Creat_Object((int)m->vertexCount, (int)m->triangleCount, "physical wheel", "");
    if (!o)
        return NULL;
    float bound = 0;
    for (size_t i = 0; i < m->vertexCount; ++i) {
        o->pointList[i].pos = gre(m->vertices[i]);
        bound = fmaxf(bound, sqrtf(YMPME_VecDot(m->vertices[i], m->vertices[i])));
    }
    for (size_t i = 0; i < m->triangleCount; ++i) {
        GRE_Polygon4d p = &o->polygonList[i];
        uint32_t a = YMPME_ConvexMesh_Index(m,3 * i), b = YMPME_ConvexMesh_Index(m,3 * i + 1), c = YMPME_ConvexMesh_Index(m,3 * i + 2);
        p->index = GRE_PolyIndex_Malloc(3 * sizeof(*p->index));
        if (!p->index) {
            YMGRE_Free_Object(o);
            return NULL;
        }
        p->num = 3;
        p->index[0] = (uint16)a;
        p->index[1] = (uint16)b;
        p->index[2] = (uint16)c;
        p->ishide = 0;
        pme_vec3 n = YMPME_VecCross(SUB(m->vertices[b], m->vertices[a]),
                                    SUB(m->vertices[c], m->vertices[a]));
        p->pN = (gre_fvector4d){n.x, n.y, n.z, 0};
    }
    o->boundType = GRE_Bounding_Sphere_R;
    o->BoundingSphereR = bound * U;
    return o;
}
int TVScene_Init(tv_scene *s, const tracked_vehicle *v) {
    memset(s, 0, sizeof(*s));
    for (int i = 0; i < TV_COLLIDERS; ++i) {
        const pme_collider *c = &v->colliders[i];
        GRE_Object4d o = NULL;
        int side = TV_LinkSide(c->body);
        GRErgb24 color =
            side >= 0      ? (c->body % TV_LINKS_PER_SIDE == TV_LINK_FIRST % TV_LINKS_PER_SIDE ? (GRErgb24){247, 125, 57}
                                                                 : (GRErgb24){116, 139, 145})
            : c->body == 0 ? (GRErgb24){64, 165, 158}
            : c->body == 1 ? (GRErgb24){39, 51, 64}
            : c->body < TV_STATIC_END
                ? (c->body < 5 ? (GRErgb24){117, 136, 151} : (GRErgb24){166, 148, 103})
            : c->type == PME_COLLIDER_MESH
                ? (i >= TV_BODIES ? (GRErgb24){188, 155, 87} : (GRErgb24){211, 172, 90})
                : (GRErgb24){124, 151, 159};
        if (c->type == PME_COLLIDER_BOX)
            o = YMGRE_MeshGener_Box(c->halfSize.x * 2 * U, c->halfSize.y * 2 * U,
                                    c->halfSize.z * 2 * U, color, "physical body", "");
        else if (c->type == PME_COLLIDER_PLANE)
            o = YMGRE_MeshGener_RectPlane(1600, 1800, 16, 18, color, "ground", "");
        else if (c->type == PME_COLLIDER_MESH)
            o = mesh(c->mesh);
        if (!add(s, o, i, 0, color))
            return 0;
    }
    if (!add(s, YMGRE_MeshGener_Box(70, 23, 75, (GRErgb24){91, 191, 179}, "cab", ""), 0, 1,
             (GRErgb24){91, 191, 179}) ||
        !add(s, YMGRE_MeshGener_Box(55, 6, 18, (GRErgb24){251, 207, 99}, "front marker", ""), 0, 2,
             (GRErgb24){251, 207, 99}))
        return 0;
    s->ambient = YMGRE_Creat_Light(0, GRE_GlobalLight, (GRErgb24){255, 255, 255}, .65f);
    s->key = YMGRE_Creat_Light(1, GRE_PointLight, (GRErgb24){255, 244, 226}, .7f);
    if (!s->ambient || !s->key)
        return 0;
    s->key->pos = (gre_fvector4d){-300, 700, -300, 1};
    s->key->proper.kc0 = 1;
    s->key->proper.kc1 = s->key->proper.kc2 = 0;
    YMGRE_List_Append(&s->lights, sizeof(*s->ambient), s->ambient);
    YMGRE_List_Append(&s->lights, sizeof(*s->key), s->key);
    s->camera = YMGRE_Creat_Camera(0, 1000, 650, 36, 36, 25, 25);
    s->workspace = YMGRE_Creat_RenderWorkspace();
    if (!s->camera || !s->workspace)
        return 0;
    YMGRE_Camera_Frustum_Init(s->camera, 1, 4000);
    return 1;
}
void TVScene_Render(tv_scene *s, const tracked_vehicle *v, int view, int wire, int forces) {
    unsigned char loaded[TV_BODIES] = {0};
    for (size_t i = 0; i < v->world.contactCount; ++i) {
        const pme_dynamic_contact *c = &v->contacts[i];
        if (c->normalImpulse > 1e-4f && ((c->bodyA > 0 && c->bodyA < TV_STATIC_END) ||
                                         (c->bodyB > 0 && c->bodyB < TV_STATIC_END))) {
            loaded[c->bodyA] = 1;
            loaded[c->bodyB] = 1;
        }
    }
    for (int i = 0; i < s->count; ++i) {
        tv_part *p = &s->parts[i];
        const pme_collider *c = &v->colliders[p->collider];
        const pme_rigid_body *b = &v->bodies[c->body];
        pme_rotation bodyR = YMPME_Quaternion_Rotation(b->orientation);
        pme_rotation r = YMPME_Quaternion_Rotation(
            YMPME_Quaternion_Multiply(b->orientation, c->localOrientation));
        pme_vec3 offset = p->decoration
                              ? (p->decoration == 1 ? V(0, .25f, -.12f) : V(0, .19f, v->colliders[0].halfSize.z - .17f))
                              : c->localPosition;
        pme_vec3 pos = ADD(b->position, YMPME_Rotation_Apply(bodyR, offset));
        GRE_Object4d o = p->object;
        o->WorldCoordinate = gre(pos);
        o->wireFrame = (uint8)(wire && (c->body == 0 || c->body >= TV_STATIC_END));
        for (int k = 0; k < o->pointNum; ++k) {
            pme_vec3 point =
                YMPME_Rotation_Apply(r, V(p->local[k].pos.x, p->local[k].pos.y, p->local[k].pos.z));
            o->pointList[k] = p->local[k];
            o->pointList[k].pos =
                (gre_fvector4d){point.x + pos.x * U, point.y + pos.y * U, point.z + pos.z * U, 1};
        }
        for (int k = 0; k < o->polygonNum; ++k) {
            pme_vec3 n =
                YMPME_Rotation_Apply(r, V(p->normals[k].x, p->normals[k].y, p->normals[k].z));
            o->polygonList[k].pN = (gre_fvector4d){n.x, n.y, n.z, 0};
            GRErgb24 color = p->color;
            if (c->type == PME_COLLIDER_MESH && fabsf(p->normals[k].x) < .00001f && k % 7 < 2)
                color = (GRErgb24){75, 89, 97};
            if (forces && TV_LinkSide(c->body) >= 0 && loaded[c->body])
                color = (GRErgb24){157, 207, 139};
            if (TV_LinkSide(c->body) >= 0 && (c->body - TV_LINK_FIRST) % TV_LINKS_PER_SIDE == 0)
                color = (GRErgb24){247, 125, 57};
            o->polygonList[k].planeColor = color;
        }
    }
    pme_vec3 center = v->bodies[0].position;
    pme_vec3 eye = view == 0   ? ADD(center, V(3.4f, 2.9f, -4.8f))
                   : view == 1 ? ADD(center, V(0, 6, -.01f))
                               : V(11, 12, -13);
    pme_vec3 target = view == 2 ? V(0, 0, 1) : ADD(center, V(0, 0, .6f));
    gre_fvector4d e = gre(eye), t = gre(target);
    YMGRE_UVNCamera_PositionInit(s->camera, &e, &t, NULL, 0);
    YMGRE_Camera_TanglePipline_RenderingWithWorkspace(s->camera, &s->lights, &s->objects,
                                                      &s->materials, s->workspace);
    gre_line3d lines[600];
    unsigned n = 0;
    for (int i = -8; i <= 8; ++i) {
        lines[n++] =
            (gre_line3d){gre(V((float)i, .004f, -8)), gre(V((float)i, .004f, 8)), {65, 83, 99}, 1};
        lines[n++] =
            (gre_line3d){gre(V(-8, .004f, (float)i)), gre(V(8, .004f, (float)i)), {65, 83, 99}, 1};
    }
    YMGRE_Camera_LineList_Rendering(s->camera, lines, n, 1);
    n = 0;
    for (int id = TV_BOGIE_FIRST; id < TV_BODIES; ++id) {
        if (v->colliders[id].type != PME_COLLIDER_MESH)
            continue;
        const pme_rigid_body *b = &v->bodies[id];
        pme_rotation r = YMPME_Quaternion_Rotation(b->orientation);
        float radius = (id - TV_BOGIE_FIRST) % TV_BOGIE_STRIDE < 3 ? v->radius : v->roadRadius;
        for (int side = 0; side < 2; ++side) {
            pme_vec3 center =
                ADD(b->position, YMPME_Rotation_Apply(r, V(side ? .145f : -.145f, 0, 0)));
            pme_vec3 tip = ADD(center, YMPME_Rotation_Apply(r, V(0, radius * .8f, 0)));
            lines[n++] = (gre_line3d){gre(center), gre(tip), {68, 76, 82}, 2};
        }
    }
    YMGRE_Camera_LineList_Rendering(s->camera, lines, n, 1);
    n = 0;
    if (forces) {
        for (int i = 0; i < TV_LINKS; ++i) {
            const pme_joint *j = &v->joints[i];
            const pme_rigid_body *b = &v->bodies[j->bodyA];
            pme_vec3 p =
                ADD(b->position, YMPME_Rotation_Apply(YMPME_Quaternion_Rotation(b->orientation),
                                                      j->localAnchorA));
            lines[n++] = (gre_line3d){
                gre(ADD(p, V(0, -.012f, 0))), gre(ADD(p, V(0, .012f, 0))), {240, 204, 100}, 2};
        }
        for (size_t i = 0; i < v->world.contactCount && n < 590; ++i) {
            const pme_dynamic_contact *c = &v->contacts[i];
            if (!((c->bodyA > 0 && c->bodyA < TV_STATIC_END) ||
                  (c->bodyB > 0 && c->bodyB < TV_STATIC_END)))
                continue;
            pme_vec3 f = c->bodyA == 0 || c->bodyA >= TV_STATIC_END ? c->reactionA.force
                                                                    : c->reactionB.force;
            float mag = sqrtf(YMPME_VecDot(f, f));
            if (mag < 1)
                continue;
            lines[n++] = (gre_line3d){gre(c->point),
                                      gre(ADD(c->point, SCALE(f, fminf(.004f, 1.f / mag)))),
                                      {117, 243, 155},
                                      2};
        }
    }
    YMGRE_Camera_LineList_Rendering(s->camera, lines, n, 0);
}
static void keep(void *p) { (void)p; }
void TVScene_Destroy(tv_scene *s) {
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
