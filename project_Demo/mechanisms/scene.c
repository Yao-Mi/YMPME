#include "scene.h"
#include "YMGRE_BasicMesh_Gener.h"
#include "YMGRE_Camera.h"
#include "YMGRE_Creat.h"
#include "YMGRE_Free.h"
#include "YMGRE_Rendering_Pipeline.h"
#include <stdlib.h>
#include <string.h>
#define U 50.0f
#define V(x, y, z) ((gre_fvector4d){(float)(x), (float)(y), (float)(z), 0})
static const GRErgb24 gold = {235, 167, 59}, blue = {64, 163, 210}, rope = {246, 218, 126},
					  steel = {137, 158, 176};
/* Persistent meshes: local vertices never accumulate transforms. */
static int shape(mech_scene *s, int type, GRErgb24 color, gre_fvector4d center, gre_fvector4d size,
				 gre_fvector4d x, gre_fvector4d y, gre_fvector4d z, int visible)
{
	int j;
	mech_part *p;
	GRE_Object4d o;
	if (s->used >= 220)
		return 0;
	p = &s->parts[s->used++];
	if (!p->object)
	{
		o = type == 0	? YMGRE_MeshGener_Box(1, 1, 1, color, "part", "")
			: type == 1 ? YMGRE_MeshGener_Cylinder(1, 1, 32, color, "pulley", "")
						: YMGRE_MeshGener_Sphere(1, 10, 16, color, "bob", "");
		if (!o)
			return 0;
		p->object = o;
		s->count = s->used;
		p->local = malloc(o->pointNum * sizeof(*p->local));
		p->normal = malloc(o->polygonNum * sizeof(*p->normal));
		if (!p->local || !p->normal)
			return 0;
		memcpy(p->local, o->pointList, o->pointNum * sizeof(*p->local));
		for (j = 0; j < o->polygonNum; ++j)
			p->normal[j] = o->polygonList[j].pN;
		p->bound = o->BoundingSphereR;
		YMGRE_List_Append(&s->objects, sizeof(*o), o);
	}
	o = p->object;
	o->isVisible = (int8)visible;
	o->WorldCoordinate = (gre_fvector4d){center.x * U, center.y * U, center.z * U, 1};
	o->BoundingSphereR = p->bound * U * fmaxf(size.x, fmaxf(size.y, size.z));
	for (j = 0; j < o->pointNum; ++j)
	{
		gre_fvector4d a = p->local[j].pos;
		a.x *= size.x;
		a.y *= size.y;
		a.z *= size.z;
		o->pointList[j] = p->local[j];
		o->pointList[j].pos = (gre_fvector4d){U * (center.x + x.x * a.x + y.x * a.y + z.x * a.z),
											  U * (center.y + x.y * a.x + y.y * a.y + z.y * a.z),
											  U * (center.z + x.z * a.x + y.z * a.y + z.z * a.z), 1};
	}
	for (j = 0; j < o->polygonNum; ++j)
	{
		gre_fvector4d a = p->normal[j];
		a.x /= size.x;
		a.y /= size.y;
		a.z /= size.z;
		o->polygonList[j].pN = V(x.x * a.x + y.x * a.y + z.x * a.z, x.y * a.x + y.y * a.y + z.y * a.z,
								 x.z * a.x + y.z * a.y + z.z * a.z);
	}
	return 1;
}
static int box(mech_scene *s, GRErgb24 c, double x, double y, double z, double w, double h, double d)
{
	return shape(s, 0, c, V(x, y, z), V(w, h, d), V(1, 0, 0), V(0, 1, 0), V(0, 0, 1), 1);
}
static int line(mech_scene *s, GRErgb24 c, pme_belt_point a, pme_belt_point b, double z, double width, double depth)
{
	double dx = b.x - a.x, dy = b.y - a.y, l = fmax(1e-9, hypot(dx, dy));
	return shape(s, 0, c, V((a.x + b.x) / 2, (a.y + b.y) / 2, z), V(width, l, depth), V(dy / l, -dx / l, 0),
				 V(dx / l, dy / l, 0), V(0, 0, 1), 1);
}
static int ball(mech_scene *s, GRErgb24 c, double x, double y, double z, double r, int visible)
{
	return shape(s, 2, c, V(x, y, z), V(r, r, r), V(1, 0, 0), V(0, 1, 0), V(0, 0, 1), visible);
}
static int wheel(mech_scene *s, double x, double y, double r, double a, GRErgb24 color)
{
	double c = cos(a), n = sin(a);
	return shape(s, 1, color, V(x, y, 0), V(r, .22, r), V(c, n, 0), V(0, 0, -1), V(-n, c, 0), 1) &&
		   shape(s, 1, steel, V(x, y, -.04), V(r * .2, .4, r * .2), V(1, 0, 0), V(0, 0, -1), V(0, 1, 0), 1) &&
		   line(s, (GRErgb24){244, 241, 216}, (pme_belt_point){x + r * .28 * c, y + r * .28 * n},
				(pme_belt_point){x + r * .88 * c, y + r * .88 * n}, -.125, .045, .025);
}
static int arc(mech_scene *s, double cx, double cy, double a, double b)
{
	for (int i = 0; i < 16; ++i)
	{
		double t = a + (b - a) * i / 16, q = a + (b - a) * (i + 1) / 16;
		if (!line(s, rope, (pme_belt_point){cx + .25 * cos(t), cy + .25 * sin(t)},
				  (pme_belt_point){cx + .25 * cos(q), cy + .25 * sin(q)}, 0, .035, .035))
			return 0;
	}
	return 1;
}
int MechScene_Init(mech_scene *s, int kind, int w, int h)
{
	memset(s, 0, sizeof(*s));
	s->kind = kind;
	s->ambient = YMGRE_Creat_Light(0, GRE_GlobalLight, (GRErgb24){255, 255, 255}, .62f);
	s->key = YMGRE_Creat_Light(1, GRE_PointLight, (GRErgb24){255, 246, 226}, .72f);
	if (!s->ambient || !s->key)
		return 0;
	s->key->pos = (gre_fvector4d){-150, 250, -250, 1};
	s->key->proper.kc0 = 1;
	s->key->proper.kc1 = s->key->proper.kc2 = 0;
	YMGRE_List_Append(&s->lights, sizeof(*s->ambient), s->ambient);
	YMGRE_List_Append(&s->lights, sizeof(*s->key), s->key);
	float beta = (float)(atan(tan(38 * MECH_PI / 180) * h / w) * 180 / MECH_PI);
	s->camera = YMGRE_Creat_Camera(0, w, h, 38, 38, beta, beta);
	s->workspace = YMGRE_Creat_RenderWorkspace();
	if (!s->camera || !s->workspace)
		return 0;
	YMGRE_Camera_Frustum_Init(s->camera, 1, 2000);
	return 1;
}
int MechScene_Render(mech_scene *s, const mechanisms_model *m, int view, int contacts)
{
	int i;
	s->used = 0;
	if (!box(s, (GRErgb24){35, 47, 61}, 0, 0, .4, 5.6, 3.9, .15))
		return 0;
	if (s->kind < 2)
	{
		const pme_belt *b = &m->belt[s->kind];
		for (i = 0; i < 2; ++i)
			if (!wheel(s, (i ? 1 : -1) * b->distance / 2, 0, b->radius[i], b->angle[i], i ? blue : gold))
				return 0;
		for (i = 0; i < PME_BELT_N; ++i)
			if (!line(s, i % 12 == 0 ? (GRErgb24){239, 120, 72} : (GRErgb24){113, 134, 145}, b->p[i],
					  b->p[(i + 1) % PME_BELT_N], 0, .07, .22))
				return 0;
		for (i = 0; i < PME_BELT_N; i += 4)
			if (!ball(s, (GRErgb24){255, 65, 66}, b->p[i].x, b->p[i].y, -.15, .04,
					  s->kind == 1 && contacts && b->contactWheel[i] >= 0))
				return 0;
	}
	else if (s->kind == 2)
	{
		const pme_pulley *h = &m->hoist;
		double y = h->loadY, top = 1.6;
		if (!box(s, steel, .2, 1.86, .1, 2.9, .12, .5) || !box(s, steel, 0, y, .2, 1.3, .11, .14) ||
			!box(s, gold, 0, y - .65, 0, .7, .64, .55) || !box(s, steel, 0, y - .3, 0, .04, .3, .04) ||
			!box(s, (GRErgb24){242, 97, 88}, 1.25, h->freeY, 0, .19, .12, .16))
			return 0;
		if (!wheel(s, -.5, y, .235, (y + .35) / .25, blue) ||
			!wheel(s, .5, y, .235, 3 * (y + .35) / .25, blue) ||
			!wheel(s, 0, top, .235, -2 * (y + .35) / .25, steel) ||
			!wheel(s, 1, top, .235, -4 * (y + .35) / .25, steel))
			return 0;
		const double xs[] = {-.75, -.25, .25, .75, 1.25};
		for (i = 0; i < 5; ++i)
			if (!line(s, rope, (pme_belt_point){xs[i], top}, (pme_belt_point){xs[i], i == 4 ? h->freeY : y}, 0, .035, .035))
				return 0;
		if (!arc(s, -.5, y, MECH_PI, 2 * MECH_PI) || !arc(s, 0, top, MECH_PI, 0) ||
			!arc(s, .5, y, MECH_PI, 2 * MECH_PI) || !arc(s, 1, top, MECH_PI, 0))
			return 0;
		if (!ball(s, gold, -.75, top, 0, .07, 1))
			return 0;
	}
	else
	{
		const swing_model *p = &m->swing;
		if (!box(s, steel, 0, 1.7, .1, 3.4, .12, .5) || !ball(s, steel, p->anchor.x, p->anchor.y, 0, .1, 1) ||
			!line(s, rope, (pme_belt_point){p->anchor.x, p->anchor.y}, (pme_belt_point){p->end.x, p->end.y}, 0, .035,
				  .035) ||
			!ball(s, blue, p->position.x, p->position.y, p->position.z, .18, 1))
			return 0;
	}
	gre_fvector4d eye = {(view ? 1.15f : 0) * U, (view ? .45f : 0) * U, -5.4f * U, 1}, target = {0, 0, 0, 1};
	YMGRE_UVNCamera_PositionInit(s->camera, &eye, &target, NULL, 0);
	YMGRE_Camera_TanglePipline_RenderingWithWorkspace(s->camera, &s->lights, &s->objects, &s->materials,
													  s->workspace);
	return 1;
}
static void keep(void *p)
{
	(void)p;
}
void MechScene_Destroy(mech_scene *s)
{
	if (s->workspace)
		YMGRE_Free_RenderWorkspace(s->workspace);
	if (s->camera)
		YMGRE_Free_Camera(s->camera);
	YMGRE_List_Clear(&s->objects, keep);
	YMGRE_List_Clear(&s->lights, keep);
	for (int i = 0; i < s->count; ++i)
	{
		free(s->parts[i].local);
		free(s->parts[i].normal);
		YMGRE_Free_Object(s->parts[i].object);
	}
	if (s->ambient)
		YMGRE_Free_Light(s->ambient);
	if (s->key)
		YMGRE_Free_Light(s->key);
}
