#include "scene.h"
#include "YMGRE_BasicMesh_Gener.h"
#include "YMGRE_Rendering_Pipeline.h"
#include "YMGRE_Camera.h"
#include "YMGRE_Creat.h"
#include "YMGRE_Free.h"
#include "YMGRE_Mem.h"
#include <stdlib.h>
#include <string.h>
#define U 50.0f
static int triangle(GRE_Object4d o, int f, int a, int b, int c, GRErgb24 color)
{
	GRE_Polygon4d p = &o->polygonList[f];
	pme_vec3 x, y, n;
	gre_fvector4d A = o->pointList[a].pos, B = o->pointList[b].pos, C = o->pointList[c].pos;
	p->index = GRE_PolyIndex_Malloc(3 * sizeof(*p->index));
	if (!p->index)
		return 0;
	p->num = 3;
	p->index[0] = (uint16)a;
	p->index[1] = (uint16)b;
	p->index[2] = (uint16)c;
	p->planeColor = color;
	p->ishide = 0;
	x = YMPME_Vec3(B.x - A.x, B.y - A.y, B.z - A.z);
	y = YMPME_Vec3(C.x - A.x, C.y - A.y, C.z - A.z);
	n = YMPME_VecCross(x, y);
	p->pN = (gre_fvector4d){n.x, n.y, n.z, 0};
	return 1;
}
static GRE_Object4d mesh(int teeth, GRErgb24 color)
{
	pme_gear_point profile[PME_GEAR_PROFILE_MAX];
	int i, n = YMPME_Gear_Profile(teeth, .12, profile, PME_GEAR_PROFILE_MAX);
	GRE_Object4d o = YMGRE_Creat_Object(2 * n + 2, 4 * n, "involute-gear", "");
	if (!o || !n)
		return NULL;
	o->pointList[0].pos = (gre_fvector4d){0, 10, 0, 1};
	o->pointList[1].pos = (gre_fvector4d){0, -10, 0, 1};
	for (i = 0; i < n; ++i)
	{
		o->pointList[2 + i].pos = (gre_fvector4d){(float)profile[i].x * U, 10, (float)profile[i].y * U, 1};
		o->pointList[2 + n + i].pos =
		    (gre_fvector4d){(float)profile[i].x * U, -10, (float)profile[i].y * U, 1};
	}
	for (i = 0; i < n; ++i)
	{
		int a = 2 + i, b = 2 + (i + 1) % n, c = a + n, d = b + n;
		GRErgb24 top = (i / 23 == 0) ? (GRErgb24){250, 225, 164} : color;
		if (!triangle(o, 4 * i, 0, b, a, top) || !triangle(o, 4 * i + 1, 1, c, d, color) ||
		    !triangle(o, 4 * i + 2, a, d, c, color) || !triangle(o, 4 * i + 3, a, b, d, color))
		{
			YMGRE_Free_Object(o);
			return NULL;
		}
	}
	o->boundType = GRE_Bounding_Sphere_R;
	o->BoundingSphereR = (float)(.12 * (teeth * .5 + 1) * U + 10);
	return o;
}
static int add(gear_scene *s, GRE_Object4d o, int owner, pme_vec3 offset, int guide)
{
	gear_part *p;
	int i;
	if (!o)
		return 0;
	if (s->count >= 20)
	{
		YMGRE_Free_Object(o);
		return 0;
	}
	p = &s->parts[s->count++];
	p->object = o;
	p->owner = owner;
	p->offset = offset;
	p->guide = guide;
	p->local = malloc((size_t)o->pointNum * sizeof(*p->local));
	p->normal = malloc((size_t)o->polygonNum * sizeof(*p->normal));
	if (!p->local || !p->normal)
		return 0;
	memcpy(p->local, o->pointList, (size_t)o->pointNum * sizeof(*p->local));
	for (i = 0; i < o->polygonNum; ++i)
		p->normal[i] = o->polygonList[i].pN;
	YMGRE_List_Append(&s->objects, sizeof(*o), o);
	return 1;
}
int GearScene_Init(gear_scene *s)
{
	return GearScene_InitMode(s, PME_GEAR_CONSTRAINT, 900, 540);
}
int GearScene_InitMode(gear_scene *s, pme_gear_mode mode, int width, int height)
{
	int i;
	memset(s, 0, sizeof(*s));
	YMPME_Gear_ResetMode(&s->pair, mode);
	for (i = 0; i < 2; ++i)
	{
		int teeth = i ? 30 : 20;
		GRErgb24 color = i ? (GRErgb24){56, 160, 200} : (GRErgb24){224, 157, 57};
		if (!add(s, mesh(teeth, color), i, YMPME_Vec3(0, 0, 0), 0) ||
		    !add(s, YMGRE_MeshGener_Cylinder(19, 28, 32, (GRErgb24){74, 86, 99}, "hub", ""), i,
		         YMPME_Vec3(0, 0, 0), 0) ||
		    !add(s, YMGRE_MeshGener_Cylinder(7, 46, 24, (GRErgb24){194, 210, 221}, "shaft", ""), i,
		         YMPME_Vec3(0, 0, 0), 0) ||
		    !add(s, YMGRE_MeshGener_Box(22, 1.5f, 4, (GRErgb24){237, 239, 221}, "rotation-mark", ""), i,
		         YMPME_Vec3(.20f, .295f, 0), 0) ||
		    !add(s,
		         YMGRE_MeshGener_Torus((float)(.12 * teeth * .5 * U), .65f, 120, 6, (GRErgb24){249, 240, 174},
		                               "pitch-circle", ""),
		         i, YMPME_Vec3(0, .225f, 0), 1))
			return 0;
	}
	if (!add(s, YMGRE_MeshGener_Box(350, 10, 225, (GRErgb24){43, 53, 64}, "base", ""), -1,
	         YMPME_Vec3(-.3f, -.53f, 0), 0))
		return 0;
	for (i = 0; i < 8; ++i)
		if (!add(s, YMGRE_MeshGener_Sphere(1.8f, 6, 8, (GRErgb24){255, 74, 75}, "tooth-contact", ""), -2,
		         YMPME_Vec3((float)i, 0, 0), 2))
			return 0;
	s->ambient = YMGRE_Creat_Light(0, GRE_GlobalLight, (GRErgb24){255, 255, 255}, .5f);
	s->key = YMGRE_Creat_Light(1, GRE_PointLight, (GRErgb24){255, 245, 224}, .85f);
	if (!s->ambient || !s->key)
		return 0;
	s->key->pos = (gre_fvector4d){-180, 350, -220, 1};
	s->key->proper.kc0 = 1;
	s->key->proper.kc1 = s->key->proper.kc2 = 0;
	YMGRE_List_Append(&s->lights, sizeof(*s->ambient), s->ambient);
	YMGRE_List_Append(&s->lights, sizeof(*s->key), s->key);
	s->camera = YMGRE_Creat_Camera(0, width, height, 38, 38,
	                               (float)(atan(tan(38 * PME_GEAR_PI / 180) * height / width) * 180 / PME_GEAR_PI),
	                               (float)(atan(tan(38 * PME_GEAR_PI / 180) * height / width) * 180 / PME_GEAR_PI));
	s->workspace = YMGRE_Creat_RenderWorkspace();
	if (!s->camera || !s->workspace)
		return 0;
	YMGRE_Camera_Frustum_Init(s->camera, 1, 2000);
	return 1;
}
void GearScene_Render(gear_scene *s, int view, int guides, int wire)
{
	int i, j;
	gre_fvector4d eye, target;
	for (i = 0; i < s->count; ++i)
	{
		gear_part *p = &s->parts[i];
		GRE_Object4d o = p->object;
		double a = p->owner < 0 ? 0 : (p->owner ? YMPME_Gear_DrivenAngle(&s->pair) : s->pair.angle);
		float c = (float)cos(a), sn = (float)sin(a),
		      cx = p->owner < 0 ? 0 : (p->owner ? (float)(-105 + s->pair.distance * U) : -105);
		pme_vec3 offset = p->offset;
		if (p->owner == -2)
		{
			int index = (int)p->offset.x;
			pme_gear_point at = s->pair.contactPoint[index];
			offset = YMPME_Vec3((float)at.x - 2.1f, .25f, (float)at.y);
		}
		o->WorldCoordinate = (gre_fvector4d){cx + offset.x * U * c - offset.z * U * sn, offset.y * U,
		                                     offset.x * U * sn + offset.z * U * c, 1};
		o->isVisible =
		    (int8)((!p->guide || guides) && (p->owner != -2 || (int)p->offset.x < s->pair.pointCount));
		o->wireFrame = (uint8)(wire && i % 5 == 0 && p->owner >= 0);
		for (j = 0; j < o->pointNum; ++j)
		{
			float x = p->local[j].pos.x + offset.x * U, z = p->local[j].pos.z + offset.z * U;
			o->pointList[j] = p->local[j];
			o->pointList[j].pos =
			    (gre_fvector4d){cx + x * c - z * sn, p->local[j].pos.y + offset.y * U, x * sn + z * c, 1};
		}
		for (j = 0; j < o->polygonNum; ++j)
		{
			gre_fvector4d n = p->normal[j];
			o->polygonList[j].pN = (gre_fvector4d){n.x * c - n.z * sn, n.y, n.x * sn + n.z * c, 0};
		}
	}
	if (view == 0)
	{
		eye = (gre_fvector4d){-10, 190, -215, 1};
		target = (gre_fvector4d){-15, 0, 0, 1};
	}
	else if (view == 1)
	{
		eye = (gre_fvector4d){-15, 285, -1, 1};
		target = (gre_fvector4d){-15, 0, 0, 1};
	}
	else
	{
		eye = (gre_fvector4d){-45, 105, -100, 1};
		target = (gre_fvector4d){-45, 0, 0, 1};
	}
	YMGRE_UVNCamera_PositionInit(s->camera, &eye, &target, NULL, 0);
	YMGRE_Camera_TanglePipline_RenderingWithWorkspace(s->camera, &s->lights, &s->objects, &s->materials,
	                                                  s->workspace);
}
static void keep(void *p)
{
	(void)p;
}
void GearScene_Destroy(gear_scene *s)
{
	int i;
	if (s->workspace)
		YMGRE_Free_RenderWorkspace(s->workspace);
	if (s->camera)
		YMGRE_Free_Camera(s->camera);
	YMGRE_List_Clear(&s->objects, keep);
	YMGRE_List_Clear(&s->lights, keep);
	for (i = 0; i < s->count; ++i)
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
