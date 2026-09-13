#include "scene.h"
#include "YMGRE_BasicMesh_Gener.h"
#include "YMGRE_Camera.h"
#include "YMGRE_Creat.h"
#include "YMGRE_Free.h"
#include "YMGRE_Mem.h"
#include "YMGRE_Rendering_Pipeline.h"
#include <stdlib.h>
#include <string.h>
#define U 30.0f
static int add(flight_scene *s, GRE_Object4d o, int owner, int motion, pme_vec3 offset,
			   pme_vec3 scale)
{
	if (!o)
		return 0;
	if (s->count >= FLIGHT_PART_MAX)
	{
		YMGRE_Free_Object(o);
		return 0;
	}
	flight_part *p = &s->parts[s->count++];
	p->object = o;
	p->owner = owner;
	p->motion = motion;
	p->offset = offset;
	/* 非均匀缩放后重算面法线，缓存一次。 */
	float bound = 0;
	for (int i = 0; i < o->pointNum; ++i)
	{
		gre_fvector4d *v = &o->pointList[i].pos;
		v->x *= scale.x;
		v->y *= scale.y;
		v->z *= scale.z;
		bound = fmaxf(bound, sqrtf(v->x * v->x + v->y * v->y + v->z * v->z));
	}
	o->boundType = GRE_Bounding_Sphere_R;
	o->BoundingSphereR = bound * U;
	for (int i = 0; i < o->polygonNum; ++i)
	{
		GRE_Polygon4d f = &o->polygonList[i];
		gre_fvector4d a = o->pointList[f->index[0]].pos, b = o->pointList[f->index[1]].pos,
					  c = o->pointList[f->index[2]].pos;
		pme_vec3 n = YMPME_VecCross(YMPME_Vec3(b.x - a.x, b.y - a.y, b.z - a.z),
									YMPME_Vec3(c.x - a.x, c.y - a.y, c.z - a.z));
		f->pN = (gre_fvector4d){n.x, n.y, n.z, 0};
	}
	p->local = malloc(o->pointNum * sizeof(*p->local));
	p->normal = malloc(o->polygonNum * sizeof(*p->normal));
	if (!p->local || !p->normal)
		return 0;
	memcpy(p->local, o->pointList, o->pointNum * sizeof(*p->local));
	for (int i = 0; i < o->polygonNum; ++i)
		p->normal[i] = o->polygonList[i].pN;
	YMGRE_List_Append(&s->objects, sizeof(*o), o);
	return 1;
}
static int box(flight_scene *s, int owner, int motion, pme_vec3 p, pme_vec3 size, GRErgb24 color)
{
	return add(s, YMGRE_MeshGener_Box(size.x, size.y, size.z, color, "airfield-part", ""), owner,
			   motion, p, YMPME_Vec3(1, 1, 1));
}
static int ellipsoid(flight_scene *s, int owner, int motion, pme_vec3 p, pme_vec3 size,
					 GRErgb24 color)
{
	return add(s, YMGRE_MeshGener_Sphere(1, 8, 12, color, "body", ""), owner, motion, p, size);
}
static GRE_Object4d wing(int side)
{
	GRE_Object4d o = YMGRE_Creat_Object(8, 12, "eagle-wing", "");
	if (!o)
		return NULL;
	const float x[4] = {0, 1.32f, 1.25f, 0}, z[4] = {.24f, .06f, -.43f, -.32f};
	for (int i = 0; i < 8; ++i)
		o->pointList[i].pos = (gre_fvector4d){side * x[i % 4], i < 4 ? .025f : -.025f, z[i % 4], 1};
	const int face[12][3] = {{0, 1, 2}, {0, 2, 3}, {4, 6, 5}, {4, 7, 6}, {0, 4, 5}, {0, 5, 1},
							 {1, 5, 6}, {1, 6, 2}, {2, 6, 7}, {2, 7, 3}, {3, 7, 4}, {3, 4, 0}};
	for (int i = 0; i < 12; ++i)
	{
		GRE_Polygon4d p = &o->polygonList[i];
		p->num = 3;
		p->index = GRE_PolyIndex_Malloc(3 * sizeof(*p->index));
		if (!p->index)
		{
			YMGRE_Free_Object(o);
			return NULL;
		}
		for (int j = 0; j < 3; ++j)
			p->index[j] = (uint16)face[i][side > 0 ? j : 2 - j];
		p->planeColor = i < 2 ? (GRErgb24){126, 82, 43} : (GRErgb24){77, 49, 31};
		p->ishide = 0;
	}
	return o;
}
int FlightScene_Reset(flight_scene *s)
{
	return YMPME_Aircraft_Init(&s->aircraft[0], PME_AIRCRAFT_DRONE, YMPME_Vec3(-6, 5, 0)) ==
			   PME_OK &&
		   YMPME_Aircraft_Init(&s->aircraft[1], PME_AIRCRAFT_EAGLE, YMPME_Vec3(0, 12, 0)) == PME_OK;
}
int FlightScene_Init(flight_scene *s)
{
	memset(s, 0, sizeof(*s));
	if (!FlightScene_Reset(s))
		return 0;
	GRErgb24 dark = {35, 45, 59}, cyan = {52, 201, 223}, gold = {246, 185, 56},
			 brown = {94, 60, 34}, cream = {235, 229, 201};
	if (!box(s, 0, 0, YMPME_Vec3(0, 0, 0), YMPME_Vec3(.42f, .18f, .62f), dark) ||
		!box(s, 0, 0, YMPME_Vec3(0, .12f, .05f), YMPME_Vec3(.3f, .08f, .32f), cyan) ||
		!ellipsoid(s, 0, 0, YMPME_Vec3(0, -.09f, .38f), YMPME_Vec3(.09f, .09f, .08f), dark))
		return 0;
	for (int i = 0; i < 4; ++i)
	{
		float x = (i & 1) ? .45f : -.45f, z = (i & 2) ? .45f : -.45f;
		if (!box(s, 0, 0, YMPME_Vec3(x * .5f, 0, z), YMPME_Vec3(.5f, .07f, .07f), dark) ||
			!box(s, 0, 0, YMPME_Vec3(0, 0, z * .5f), YMPME_Vec3(.07f, .07f, .5f), dark) ||
			!add(s, YMGRE_MeshGener_Cylinder(.08f, .16f, 12, cyan, "motor", ""), 0, 0,
				 YMPME_Vec3(x, .05f, z), YMPME_Vec3(1, 1, 1)) ||
			!box(s, 0, i + 1, YMPME_Vec3(x, .15f, z), YMPME_Vec3(.62f, .012f, .055f),
				 i < 2 ? cyan : gold) ||
			!box(s, 0, i + 1, YMPME_Vec3(x, .15f, z), YMPME_Vec3(.055f, .012f, .62f),
				 i < 2 ? cyan : gold))
			return 0;
	}
	for (int side = -1; side <= 1; side += 2)
		if (!box(s, 0, 0, YMPME_Vec3(side * .23f, -.24f, 0), YMPME_Vec3(.04f, .04f, .65f), dark) ||
			!box(s, 0, 0, YMPME_Vec3(side * .23f, -.14f, 0), YMPME_Vec3(.035f, .22f, .04f), dark))
			return 0;
	if (!ellipsoid(s, 1, 0, YMPME_Vec3(0, 0, 0), YMPME_Vec3(.22f, .24f, .55f), brown) ||
		!ellipsoid(s, 1, 0, YMPME_Vec3(0, .12f, .52f), YMPME_Vec3(.2f, .21f, .27f), cream) ||
		!ellipsoid(s, 1, 0, YMPME_Vec3(0, .07f, .8f), YMPME_Vec3(.09f, .065f, .17f), gold))
		return 0;
	for (int side = -1; side <= 1; side += 2)
	{
		if (!ellipsoid(s, 1, 0, YMPME_Vec3(side * .175f, .19f, .63f),
					   YMPME_Vec3(.028f, .035f, .036f), dark) ||
			!add(s, wing(side), 1, side > 0 ? 5 : 6, YMPME_Vec3(side * .14f, 0, .03f),
				 YMPME_Vec3(1, 1, 1)))
			return 0;
		for (int i = 0; i < 6; ++i)
			if (!box(s, 1, side > 0 ? 5 : 6, YMPME_Vec3(side * .14f, 0, .03f),
					 YMPME_Vec3(.12f, .025f, .48f), i % 2 ? brown : (GRErgb24){116, 79, 44}))
				return 0;
		/* 羽片缓存相对翼根，随整翼共同振动。 */
		for (int i = 0; i < 6; ++i)
		{
			flight_part *p = &s->parts[s->count - 6 + i];
			for (int j = 0; j < p->object->pointNum; ++j)
			{
				p->local[j].pos.x += side * (.65f + .145f * i);
				p->local[j].pos.z -= .46f + .06f * i;
			}
			p->object->BoundingSphereR = 2 * U;
		}
		if (!box(s, 1, 0, YMPME_Vec3(side * .085f, -.28f, -.27f), YMPME_Vec3(.055f, .07f, .18f),
				 gold))
			return 0;
	}
	for (int i = -2; i <= 2; ++i)
		if (!box(s, 1, 0, YMPME_Vec3(i * .09f, -.04f, -.67f - fabsf((float)i) * .04f),
				 YMPME_Vec3(.095f, .028f, .58f), cream))
			return 0;
	GRE_Object4d ground =
		YMGRE_MeshGener_RectPlane(180, 180, 60, 60, (GRErgb24){107, 132, 85}, "meadow", "");
	if (!ground)
		return 0;
	for (int i = 0; i < ground->polygonNum; ++i)
	{
		int cell = i / 2;
		ground->polygonList[i].planeColor =
			((cell / 60 + cell % 60) & 1) ? (GRErgb24){107, 132, 85} : (GRErgb24){113, 138, 90};
	}
	if (!add(s, ground, -1, 0, YMPME_Vec3(15, 0, 10), YMPME_Vec3(1, 1, 1)))
		return 0;
	if (!box(s, -1, 0, YMPME_Vec3(-6, .018f, 0), YMPME_Vec3(3, .025f, 3), dark) ||
		!box(s, -1, 0, YMPME_Vec3(-6, .04f, 0), YMPME_Vec3(.2f, .02f, 1.7f), cream))
		return 0;
	for (int i = 0; i < 16; ++i)
	{
		float angle = i * 6.2831853f / 16, x = 15 + 44 * cosf(angle), z = 8 + 44 * sinf(angle),
			  height = 3 + (i % 3);
		if (!add(s, YMGRE_MeshGener_Cone(1.8f, height, 8, (GRErgb24){47, 85, 61}, "pine", ""), -1,
				 0, YMPME_Vec3(x, height * .5f + .8f, z), YMPME_Vec3(1, 1, 1)) ||
			!box(s, -1, 0, YMPME_Vec3(x, .6f, z), YMPME_Vec3(.25f, 1.2f, .25f), brown))
			return 0;
	}
	for (int i = 0; i < 2; ++i)
		if (!ellipsoid(s, i, 7, YMPME_Vec3(0, 0, 0), YMPME_Vec3(i ? 1.4f : .8f, .008f, .6f),
					   (GRErgb24){55, 69, 47}))
			return 0;
	s->ambient = YMGRE_Creat_Light(0, GRE_GlobalLight, (GRErgb24){255, 255, 255}, .7f);
	s->key = YMGRE_Creat_Light(1, GRE_PointLight, (GRErgb24){255, 247, 220}, .7f);
	if (!s->ambient || !s->key)
		return 0;
	s->key->pos = (gre_fvector4d){-2000, 5000, -2000, 1};
	s->key->proper.kc0 = 1;
	s->key->proper.kc1 = s->key->proper.kc2 = 0;
	YMGRE_List_Append(&s->lights, sizeof(*s->ambient), s->ambient);
	YMGRE_List_Append(&s->lights, sizeof(*s->key), s->key);
	for (int i = 0; i < 2; ++i)
	{
		float beta = atanf(tanf(32 * 3.14159265f / 180) * 450 / 600) * 180 / 3.14159265f;
		s->camera[i] = YMGRE_Creat_Camera(i, 600, 450, 32, 32, beta, beta);
		s->workspace[i] = YMGRE_Creat_RenderWorkspace();
		if (!s->camera[i] || !s->workspace[i])
			return 0;
		YMGRE_Camera_Frustum_Init(s->camera[i], .1f * U, 400 * U);
	}
	return 1;
}
static pme_vec3 animate(const flight_part *p, const pme_aircraft *a, pme_vec3 v)
{
	float angle = 0;
	if (p->motion >= 1 && p->motion <= 4)
	{
		angle = a->rotorPhase[p->motion - 1];
		return YMPME_Vec3(v.x * cosf(angle) + v.z * sinf(angle), v.y,
						  -v.x * sinf(angle) + v.z * cosf(angle));
	}
	if (p->motion == 5 || p->motion == 6)
	{
		angle = (p->motion == 5 ? 1 : -1) * a->wingAngle;
		return YMPME_Vec3(v.x * cosf(angle) - v.y * sinf(angle),
						  v.x * sinf(angle) + v.y * cosf(angle), v.z);
	}
	return v;
}
void FlightScene_Render(flight_scene *s, int view)
{
	for (int i = 0; i < s->count; ++i)
	{
		flight_part *p = &s->parts[i];
		GRE_Object4d o = p->object;
		const pme_aircraft *a = p->owner < 0 ? NULL : &s->aircraft[p->owner];
		pme_vec3 origin = p->offset;
		if (a)
			origin = p->motion == 7
						 ? YMPME_Vec3(a->position.x, .018f, a->position.z)
						 : YMPME_VecAdd(a->position, YMPME_Aircraft_Rotate(a, p->offset));
		o->WorldCoordinate = (gre_fvector4d){origin.x * U, origin.y * U, origin.z * U, 1};
		for (int j = 0; j < o->pointNum; ++j)
		{
			gre_fvector4d v = p->local[j].pos;
			pme_vec3 at = YMPME_Vec3(v.x, v.y, v.z);
			if (a && p->motion != 7)
				at = YMPME_Aircraft_Rotate(a, animate(p, a, at));
			at = YMPME_VecAdd(origin, at);
			o->pointList[j] = p->local[j];
			o->pointList[j].pos = (gre_fvector4d){at.x * U, at.y * U, at.z * U, 1};
		}
		for (int j = 0; j < o->polygonNum; ++j)
		{
			gre_fvector4d v = p->normal[j];
			pme_vec3 n = YMPME_Vec3(v.x, v.y, v.z);
			if (a && p->motion != 7)
				n = YMPME_Aircraft_Rotate(a, animate(p, a, n));
			o->polygonList[j].pN = (gre_fvector4d){n.x, n.y, n.z, 0};
		}
	}
	for (int i = 0; i < 2; ++i)
	{
		pme_aircraft *a = &s->aircraft[i];
		pme_vec3 offset;
		if (view == 0)
			offset = YMPME_Aircraft_Rotate(a, YMPME_Vec3(i ? -3.2f : -2.6f, 1.9f, -4.0f));
		else if (view == 1)
			offset = YMPME_Vec3(0, 8, -.3f);
		else
			offset = YMPME_Vec3(6, 3, 5);
		pme_vec3 eye = YMPME_VecAdd(a->position, offset), target = a->position;
		gre_fvector4d e = {eye.x * U, eye.y * U, eye.z * U, 1},
					  t = {target.x * U, target.y * U, target.z * U, 1};
		YMGRE_UVNCamera_PositionInit(s->camera[i], &e, &t, NULL, 0);
		YMGRE_Camera_TanglePipline_RenderingWithWorkspace(s->camera[i], &s->lights, &s->objects,
														  &s->materials, s->workspace[i]);
		GRE_RenderTarget rt = YMGRE_Camera_GetRenderTarget(s->camera[i]);
		for (int y = 0; y < rt->height; ++y)
		{
			GRErgb24 sky = {(uint8)(121 + 44 * y / rt->height), (uint8)(178 + 28 * y / rt->height),
							(uint8)(213 + 11 * y / rt->height)};
			for (int x = 0; x < rt->width; ++x)
			{
				int k = y * rt->width + x;
				if (rt->zbuff[k] >= s->camera[i]->frustum.Zfar)
					rt->data[k] = GRE_FramePixel_From_RGB24(sky);
			}
		}
	}
}
static void keep(void *p) { (void)p; }
void FlightScene_Destroy(flight_scene *s)
{
	for (int i = 0; i < 2; ++i)
	{
		if (s->workspace[i])
			YMGRE_Free_RenderWorkspace(s->workspace[i]);
		if (s->camera[i])
			YMGRE_Free_Camera(s->camera[i]);
	}
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
	memset(s, 0, sizeof(*s));
}
