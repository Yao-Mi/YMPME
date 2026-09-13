#include "flight.h"
#include "YMGRE_BasicMesh_Gener.h"
#include "YMGRE_Camera.h"
#include "YMGRE_Creat.h"
#include "YMGRE_Free.h"
#include <string.h>
#define V YMPME_Vec3
#define PI 3.14159265358979323846f
static int add(owl_flight *s, GRE_Object4d o, pme_vec3 at)
{
	if (!o)
		return 0;
	if (s->sceneryCount == OWL_WORLD_OBJECTS)
	{
		YMGRE_Free_Object(o);
		return 0;
	}
	int n = s->sceneryCount++;
	s->scenery[n] = o;
	for (int j = 0; j < o->pointNum; j++)
	{
		o->pointList[j].pos.x += at.x;
		o->pointList[j].pos.y += at.y;
		o->pointList[j].pos.z += at.z;
	}
	o->WorldCoordinate = (gre_fvector4d){at.x, at.y, at.z, 1};
	o->renderMode = GRE_RenderMode_Face;
	if (!YMGRE_Object_EnableVertexAttributes(o))
		return 0;
	s->sceneryNodes[n] = (gre_listnode){sizeof(*o), o, s->visual.objectNode.next};
	s->visual.objectNode.next = &s->sceneryNodes[n];
	s->visual.objects.len++;
	return 1;
}
int OwlFlight_Reset(owl_flight *s)
{
	s->flapBlend = 1;
	return YMPME_Aircraft_Init(&s->aircraft, PME_AIRCRAFT_EAGLE, V(0, 18, 0)) == PME_OK;
}
int OwlFlight_Init(owl_flight *s, const char *root)
{
	memset(s, 0, sizeof(*s));
	if (!(root ? Demo_InitFiles(&s->visual, root, 0) : Demo_Init(&s->visual)) ||
		!OwlFlight_Reset(s))
		return 0;
	/* Choose the widest sampled pose once as the glide pose, using the imported clip. */
	float widest = -1;
	for (unsigned f = 0; f + 1 < s->visual.animation.info.frameCount; f++)
	{
		if (!Demo_Update(&s->visual, (float)f))
			return 0;
		float span = s->visual.object->BoundingBoxMax.x - s->visual.object->BoundingBoxMin.x;
		if (span > widest)
		{
			widest = span;
			s->glideFrame = (int)f;
		}
	}
	GRE_Object4d g =
		YMGRE_MeshGener_RectPlane(400, 400, 40, 40, (GRErgb24){93, 124, 72}, "meadow", "");
	if (!g)
		return 0;
	for (int p = 0; p < g->polygonNum; p++)
	{
		int c = p / 2;
		g->polygonList[p].planeColor =
			((c / 40 + c % 40) & 1) ? (GRErgb24){93, 124, 72} : (GRErgb24){112, 140, 86};
	}
	if (!add(s, g, V(0, 0, 0)))
		return 0;
	for (int i = 0; i < 12; i++)
	{
		float angle = 2 * PI * i / 12, x = 34 * cosf(angle), z = 34 * sinf(angle), h = 3 + (i % 4);
		if (!add(s, YMGRE_MeshGener_Cone(2, h, 10, (GRErgb24){41, 83, 52}, "pine", ""),
				 V(x, h * .5f + 1, z)) ||
			!add(s, YMGRE_MeshGener_Box(.4f, 2, .4f, (GRErgb24){93, 62, 38}, "trunk", ""),
				 V(x, 1, z)))
			return 0;
	}
	YMGRE_Camera_Frustum_Init(s->visual.camera, .08f, 300);
	return 1;
}
int OwlFlight_Step(owl_flight *s, const pme_aircraft_command *c, pme_vec3 wind)
{
	if (YMPME_Aircraft_Step(&s->aircraft, c, wind, 1.f / 240) != PME_OK)
		return 0;
	s->flapBlend += (c->power - s->flapBlend) * (1 - expf(-1.f / (240 * .15f)));
	return 1;
}
static int pose(owl_flight *s)
{
	demo_scene *v = &s->visual;
	const unsigned count = v->asset.info.boneCount, last = v->animation.info.frameCount - 1;
	float phase = s->aircraft.wingPhase / (2 * PI);
	phase -= floorf(phase);
	float frame = phase * last;
	unsigned a = (unsigned)frame, b = a < last ? a + 1 : a;
	float t = frame - a;
	/* Imported model faces +Z. Its approximately 6-unit span becomes 2.7 m. */
	ske_mat34 world;
	if (YMSKE_Mat_TRS(
			(ske_vec3){s->aircraft.position.x, s->aircraft.position.y, s->aircraft.position.z},
			(ske_quat){s->aircraft.q[0], s->aircraft.q[1], s->aircraft.q[2], s->aircraft.q[3]},
			(ske_vec3){.45f, .45f, .45f}, &world) != SKE_OK)
		return 0;
	for (int r = 0; r < 3; r++)
		world.m[r][3] -= world.m[r][1] * .65f;
	for (unsigned i = 0; i < count; i++)
	{
		for (int r = 0; r < 3; r++)
			for (int c = 0; c < 4; c++)
			{
				float animated = v->animation.samples[a * count + i].m[r][c] * (1 - t) +
								 v->animation.samples[b * count + i].m[r][c] * t;
				float glide = v->animation.samples[s->glideFrame * count + i].m[r][c];
				v->poseBones[i].local.m[r][c] = glide + (animated - glide) * s->flapBlend;
			}
		if (v->bones[i].parent < 0)
			v->poseBones[i].local = YMSKE_Mat_Multiply(world, v->poseBones[i].local);
	}
	return YMSKE_Pose_Update(&v->pose) == SKE_OK &&
		   YMSKE_GRE_Update(&v->binding, &v->pose) == SKE_OK;
}
int OwlFlight_Render(owl_flight *s, int view, int wire, int bones)
{
	if (!pose(s))
		return 0;
	demo_scene *v = &s->visual;
	pme_vec3 p = s->aircraft.position, f = YMPME_Aircraft_Rotate(&s->aircraft, V(0, 0, 1));
	float yaw = atan2f(f.x, f.z);
	pme_vec3 off = view == 0   ? V(-2.1f, 1.3f, -4.8f)
				   : view == 1 ? V(0, 7, -.1f)
							   : V(2.5f, 1.0f, 4.7f);
	if (view != 1)
		off =
			V(off.x * cosf(yaw) + off.z * sinf(yaw), off.y, -off.x * sinf(yaw) + off.z * cosf(yaw));
	gre_fvector4d eye = {p.x + off.x, p.y + off.y, p.z + off.z, 1}, at = {p.x, p.y, p.z, 1};
	YMGRE_UVNCamera_PositionInit(v->camera, &eye, &at, NULL, 0);
	v->key->pos = (gre_fvector4d){p.x + 20, p.y + 30, p.z + 20, 1};
	/* Recenter checkerboard in 20 m increments, preserving its pattern phase. */
	GRE_Object4d g = s->scenery[0];
	float gx = 20 * floorf(p.x / 20), gz = 20 * floorf(p.z / 20);
	float dx = gx - g->WorldCoordinate.x, dz = gz - g->WorldCoordinate.z;
	for (int i = 0; i < g->pointNum; i++)
	{
		g->pointList[i].pos.x += dx;
		g->pointList[i].pos.z += dz;
		g->pointList_wN[i].base.pos = g->pointList[i].pos;
	}
	g->WorldCoordinate = (gre_fvector4d){gx, 0, gz, 1};
	v->object->wireFrame = !!wire;
	YMGRE_Camera_TanglePipline_wN(v->camera, &v->lights, &v->objects, &v->materials, &v->workspace);
	GRE_RenderTarget target = YMGRE_Camera_GetRenderTarget(v->camera);
	for (int y = 0; y < target->height; y++)
		for (int x = 0; x < target->width; x++)
		{
			size_t i = (size_t)y * target->width + x;
			if (target->zbuff[i] >= v->camera->frustum.Zfar)
				target->data[i] = GRE_FramePixel_From_RGB24(
					(GRErgb24){(uint8)(110 + 50 * y / target->height),
							   (uint8)(167 + 40 * y / target->height), 220});
		}
	if (bones)
	{
		gre_line3d lines[DEMO_BONES];
		unsigned n = 0;
		for (unsigned i = 0; i < v->asset.info.boneCount; i++)
		{
			const char *name = v->asset.names[i].text;
			if (strncmp(name, "Spine", 5) && strncmp(name, "Head", 4) &&
				strncmp(name, "Bicep", 5) && strncmp(name, "ForeArm", 7) &&
				strncmp(name, "WingTip", 7))
				continue;
			ske_vec3 a = YMSKE_Mat_Point(v->poseBones[i].global, (ske_vec3){0, 0, 0}),
					 b = YMSKE_Mat_Point(v->poseBones[i].global, v->asset.tails[i]);
			lines[n++] = (gre_line3d){{a.x, a.y, a.z, 1}, {b.x, b.y, b.z, 1}, {90, 240, 255}, 1};
		}
		YMGRE_Camera_LineList_Rendering(v->camera, lines, n, 0);
	}
	return 1;
}
void OwlFlight_Destroy(owl_flight *s)
{
	for (int i = 0; i < s->sceneryCount; i++)
		YMGRE_Free_Object(s->scenery[i]);
	Demo_Destroy(&s->visual);
	memset(s, 0, sizeof(*s));
}
