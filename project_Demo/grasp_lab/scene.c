#include "scene.h"
#include "YMGRE_BasicMesh_Gener.h"
#include "YMGRE_Rendering_Pipeline.h"
#include "YMGRE_Camera.h"
#include "YMGRE_Creat.h"
#include "YMGRE_Free.h"
#include "YMGRE_Mem.h"
#include <stdlib.h>
#include <string.h>
#define U 100.f
#define V YMPME_Vec3
static int add(grasp_scene *s, GRE_Object4d o, int body, pme_vec3 offset)
{
	if (!o)
		return 0;
	if (s->count == 12)
	{
		YMGRE_Free_Object(o);
		return 0;
	}
	grasp_part *p = &s->parts[s->count++];
	p->object = o;
	p->body = body;
	p->shape = -1;
	p->offset = offset;
	p->local = malloc((size_t)o->pointNum * sizeof(*p->local));
	p->normals = malloc((size_t)o->polygonNum * sizeof(*p->normals));
	p->colors = malloc((size_t)o->polygonNum * sizeof(*p->colors));
	if (!p->local || !p->normals || !p->colors)
		return 0;
	memcpy(p->local, o->pointList, (size_t)o->pointNum * sizeof(*p->local));
	for (int i = 0; i < o->polygonNum; ++i)
	{
		p->normals[i] = o->polygonList[i].pN;
		p->colors[i] = o->polygonList[i].planeColor;
	}
	YMGRE_List_Append(&s->objects, sizeof(*o), o);
	return 1;
}
static int box(grasp_scene *s, pme_vec3 size, GRErgb24 c, int owner, pme_vec3 offset)
{
	return add(s, YMGRE_MeshGener_Box(size.x * U, size.y * U, size.z * U, c, "gripper", ""), owner,
			   offset);
}
static GRE_Object4d polyMesh(const grasp_geometry *g)
{
	GRE_Object4d o = YMGRE_Creat_Object((int)g->mesh.vertexCount, (int)g->mesh.triangleCount,
										"convex object", "");
	if (!o)
		return NULL;
	for (size_t i = 0; i < g->mesh.vertexCount; ++i)
	{
		pme_vec3 v = g->vertices[i];
		o->pointList[i].pos = (gre_fvector4d){v.x * U, v.y * U, v.z * U, 1};
	}
	for (size_t i = 0; i < g->mesh.triangleCount; ++i)
	{
		GRE_Polygon4d p = &o->polygonList[i];
		uint32_t a = g->indices[i * 3], b = g->indices[i * 3 + 1], c = g->indices[i * 3 + 2];
		pme_vec3 n = YMPME_VecCross(YMPME_VecSub(g->vertices[b], g->vertices[a]),
									YMPME_VecSub(g->vertices[c], g->vertices[a]));
		if (YMPME_VecDot(n, g->vertices[a]) < 0)
		{
			uint32_t t = b;
			b = c;
			c = t;
			n = YMPME_VecScale(n, -1);
		}
		p->index = GRE_PolyIndex_Malloc(3 * sizeof(*p->index));
		if (!p->index)
		{
			YMGRE_Free_Object(o);
			return NULL;
		}
		p->num = 3;
		p->index[0] = (uint16)a;
		p->index[1] = (uint16)b;
		p->index[2] = (uint16)c;
		p->ishide = 0;
		p->planeColor = fabsf(n.y) > .5f * sqrtf(YMPME_VecDot(n, n)) ? (GRErgb24){198, 221, 235}
																	 : (GRErgb24){84, 134, 174};
		p->pN = (gre_fvector4d){n.x, n.y, n.z, 0};
	}
	o->boundType = GRE_Bounding_Sphere_R;
	o->BoundingSphereR = .4f * U;
	return o;
}
int GraspScene_Init(grasp_scene *s)
{
	memset(s, 0, sizeof(*s));
	if (!add(s, YMGRE_MeshGener_RectPlane(6 * U, 4 * U, 12, 8, (GRErgb24){51, 64, 77}, "floor", ""),
			 -1, V(0, 0, 0)) ||
		!box(s, V(.12f, 2, .15f), (GRErgb24){58, 82, 105}, -1, V(-.58f, 1, .27f)) ||
		!box(s, V(.12f, 2, .15f), (GRErgb24){58, 82, 105}, -1, V(.58f, 1, .27f)) ||
		!box(s, V(.9f, .12f, .30f), (GRErgb24){133, 159, 179}, 0, V(0, 0, 0)) ||
		!box(s, V(.1f, .8f, .30f), (GRErgb24){44, 170, 207}, 1, V(0, 0, 0)) ||
		!box(s, V(.1f, .8f, .30f), (GRErgb24){218, 155, 60}, 2, V(0, 0, 0)))
		return 0;
	for (int i = 1; i <= 2; ++i)
		if (!add(s, YMGRE_MeshGener_Sphere(.065f * U, 8, 12, (GRErgb24){236, 97, 92}, "hinge", ""),
				 i, V(0, .40f, 0)))
			return 0;
	if (!add(s, YMGRE_MeshGener_Sphere(.22f * U, 8, 16, (GRErgb24){206, 219, 229}, "free ball", ""),
			 3, V(0, 0, 0)))
		return 0;
	grasp_part *ball = &s->parts[s->count - 1];
	ball->shape = 0;
	for (int i = 0; i < ball->object->polygonNum; ++i)
	{
		const gre_fvector4d n = ball->normals[i];
		ball->colors[i] = n.x * n.z > 0 ? (GRErgb24){186, 207, 230} : (GRErgb24){57, 103, 150};
	}
	for (int shape = 1; shape <= 2; ++shape)
	{
		grasp_geometry g;
		if (!Grasp_Geometry(&g, shape, .3f) || !add(s, polyMesh(&g), 3, V(0, 0, 0)))
			return 0;
		s->parts[s->count - 1].shape = shape;
	}
	s->ambient = YMGRE_Creat_Light(0, GRE_GlobalLight, (GRErgb24){255, 255, 255}, .65f);
	s->key = YMGRE_Creat_Light(1, GRE_PointLight, (GRErgb24){255, 244, 231}, .7f);
	if (!s->ambient || !s->key)
		return 0;
	s->key->pos = (gre_fvector4d){-200, 350, -260, 1};
	s->key->proper.kc0 = 1;
	s->key->proper.kc1 = s->key->proper.kc2 = 0;
	YMGRE_List_Append(&s->lights, sizeof(*s->ambient), s->ambient);
	YMGRE_List_Append(&s->lights, sizeof(*s->key), s->key);
	s->camera = YMGRE_Creat_Camera(0, 840, 640, 31, 31, 26, 26);
	s->workspace = YMGRE_Creat_RenderWorkspace();
	s->maps[0] = YMGRE_Creat_RenderTarget(144, 288);
	s->maps[1] = YMGRE_Creat_RenderTarget(144, 288);
	if (!s->camera || !s->workspace || !s->maps[0] || !s->maps[1])
		return 0;
	YMGRE_Camera_Frustum_Init(s->camera, 1, 2000);
	return 1;
}
static GRErgb24 heat(const grasp_lab *state, int sensor, size_t cell, int pressure)
{
	const pme_surface_taxel *c = &state->cells[sensor][cell];
	if (YMPME_VecDot(c->force, c->force) < 1e-10f)
		return (GRErgb24){20, 31, 46};
	if (pressure)
	{
		float f = YMPME_Clamp(c->pressure / 15000.f, 0, 1);
		return (GRErgb24){(uint8)(255 * f), (uint8)(220 * sqrtf(f)), (uint8)(35 + 50 * f)};
	}
	pme_vec3 f = YMPME_Rotation_InverseApply(state->sampleRotation[sensor], c->force);
	float u = f.z, v = f.y;
	return (GRErgb24){(uint8)(255 * YMPME_Clamp(c->normalForce / 4, 0, 1)),
					  (uint8)(128 + 127 * YMPME_Clamp(u / 2, -1, 1)),
					  (uint8)(128 + 127 * YMPME_Clamp(v / 2, -1, 1))};
}
static gre_fvector4d gre(pme_vec3 p) { return (gre_fvector4d){p.x * U, p.y * U, p.z * U, 1}; }
static void arrow(gre_line3d *lines, unsigned *count, pme_vec3 start, pme_vec3 force,
				  GRErgb24 color)
{
	float length = sqrtf(YMPME_VecDot(force, force));
	if (length < .02f)
		return;
	pme_vec3 direction = YMPME_VecScale(force, 1 / length);
	pme_vec3 end = YMPME_VecAdd(start, YMPME_VecScale(direction, fminf(.4f, length * .045f)));
	pme_vec3 side = YMPME_VecCross(direction, V(0, 0, 1));
	if (YMPME_VecDot(side, side) < .01f)
		side = YMPME_VecCross(direction, V(0, 1, 0));
	side = YMPME_VecScale(side, .025f / sqrtf(YMPME_VecDot(side, side)));
	pme_vec3 base = YMPME_VecSub(end, YMPME_VecScale(direction, .04f));
	lines[(*count)++] = (gre_line3d){gre(start), gre(end), color, 3};
	lines[(*count)++] = (gre_line3d){gre(end), gre(YMPME_VecAdd(base, side)), color, 2};
	lines[(*count)++] = (gre_line3d){gre(end), gre(YMPME_VecSub(base, side)), color, 2};
}
void GraspScene_Render(grasp_scene *s, const grasp_lab *state, int view, int wire, int pressure)
{
	for (int i = 0; i < s->count; ++i)
	{
		grasp_part *p = &s->parts[i];
		GRE_Object4d o = p->object;
		pme_rotation r = YMPME_Rotation_Identity();
		pme_vec3 center = p->offset;
		if (p->body >= 0)
		{
			r = YMPME_Quaternion_Rotation(state->bodies[p->body].orientation);
			center =
				YMPME_VecAdd(state->bodies[p->body].position, YMPME_Rotation_Apply(r, p->offset));
		}
		o->WorldCoordinate = gre(center);
		o->wireFrame = (uint8)(wire && p->body == 3);
		o->isVisible = (int8)(p->shape < 0 || p->shape == state->shape);
		for (int k = 0; k < o->pointNum; ++k)
		{
			pme_vec3 v =
				YMPME_Rotation_Apply(r, V(p->local[k].pos.x, p->local[k].pos.y, p->local[k].pos.z));
			o->pointList[k] = p->local[k];
			o->pointList[k].pos =
				(gre_fvector4d){v.x + center.x * U, v.y + center.y * U, v.z + center.z * U, 1};
		}
		for (int k = 0; k < o->polygonNum; ++k)
		{
			pme_vec3 n =
				YMPME_Rotation_Apply(r, V(p->normals[k].x, p->normals[k].y, p->normals[k].z));
			o->polygonList[k].pN = (gre_fvector4d){n.x, n.y, n.z, 0};
			o->polygonList[k].planeColor = p->colors[k];
		}
	}
	gre_fvector4d eye = view == 0	? gre(V(1.5f, 1.7f, 3.5f))
						: view == 1 ? gre(V(0, 1.1f, 3.7f))
									: gre(V(2.6f, 1.3f, -.3f));
	gre_fvector4d target = gre(V(.1f, .9f, 0));
	YMGRE_UVNCamera_PositionInit(s->camera, &eye, &target, NULL, 0);
	YMGRE_Camera_TanglePipline_RenderingWithWorkspace(s->camera, &s->lights, &s->objects,
													  &s->materials, s->workspace);
	gre_line3d lines[256];
	unsigned count = 0;
	for (int i = -6; i <= 6; ++i)
	{
		lines[count++] =
			(gre_line3d){gre(V(i * .5f, .001f, -2)), gre(V(i * .5f, .001f, 2)), {80, 97, 111}, 1};
		lines[count++] =
			(gre_line3d){gre(V(-3, .001f, i / 3.f)), gre(V(3, .001f, i / 3.f)), {80, 97, 111}, 1};
	}
	YMGRE_Camera_LineList_Rendering(s->camera, lines, count, 1);
	count = 0;
	for (size_t i = 0; i < state->world.contactCount; ++i)
	{
		const pme_dynamic_contact *c = &state->contacts[i];
		arrow(lines, &count, c->point,
			  c->bodyB == GRASP_OBJECT ? c->reactionB.force : c->reactionA.force,
			  (GRErgb24){97, 255, 186});
		arrow(lines, &count, c->point,
			  c->bodyB == GRASP_OBJECT ? c->reactionA.force : c->reactionB.force,
			  (GRErgb24){255, 166, 91});
	}
	/* Show each loaded skin cell on its actual body, including the far side.
	 * This is a contact marker; force magnitude is encoded only in the atlas. */
	for (int sensor = 0; sensor < 2; ++sensor)
		for (size_t k = 0; k < Grasp_CellCount(sensor); ++k)
		{
			if (YMPME_VecDot(state->cells[sensor][k].force, state->cells[sensor][k].force) < 1e-8f)
				continue;
			pme_vec3 p = state->patches[sensor][k].position;
			lines[count++] = (gre_line3d){gre(YMPME_VecAdd(p, V(-.022f, 0, 0))),
										  gre(YMPME_VecAdd(p, V(.022f, 0, 0))),
										  {255, 253, 185},
										  3};
			lines[count++] = (gre_line3d){gre(YMPME_VecAdd(p, V(0, -.022f, 0))),
										  gre(YMPME_VecAdd(p, V(0, .022f, 0))),
										  {255, 253, 185},
										  3};
		}
	YMGRE_Camera_LineList_Rendering(s->camera, lines, count, 0);
	for (int sensor = 0; sensor < 2; ++sensor)
	{
		GRE_RenderTarget t = s->maps[sensor];
		int cellSize = 18, width = 8;
		for (int y = 0; y < t->height; ++y)
			for (int x = 0; x < t->width; ++x)
			{
				size_t k = (size_t)(y / cellSize) * width + x / cellSize;
				GRErgb24 c = heat(state, sensor, k, pressure);
				if (x % cellSize == 0 || y % cellSize == 0)
					c = (GRErgb24){43, 59, 73};
				else if ((x % cellSize == 1 || y % cellSize == 1 || x % cellSize == cellSize - 1 ||
						  y % cellSize == cellSize - 1) &&
						 YMPME_VecDot(state->cells[sensor][k].force,
									  state->cells[sensor][k].force) > 1e-8f)
					c = (GRErgb24){255, 245, 210};
				t->data[y * t->width + x] = GRE_FramePixel_From_RGB24(c);
			}
	}
}
static void keep(void *p) { (void)p; }
void GraspScene_Destroy(grasp_scene *s)
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
		free(s->parts[i].normals);
		free(s->parts[i].colors);
		YMGRE_Free_Object(s->parts[i].object);
	}
	if (s->ambient)
		YMGRE_Free_Light(s->ambient);
	if (s->key)
		YMGRE_Free_Light(s->key);
	for (int i = 0; i < 2; ++i)
		if (s->maps[i])
			YMGRE_Free_RenderTarget(s->maps[i]);
}
