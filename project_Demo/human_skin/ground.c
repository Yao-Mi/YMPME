#include "rig.h"
#include "YMGRE_Creat.h"
#include "YMGRE_Free.h"
#include "YMGRE_BasicMesh_Gener.h"
#include <string.h>
#define V YMPME_Vec3
static pme_vec3 add(pme_vec3 a, pme_vec3 b) { return YMPME_VecAdd(a, b); }
static pme_vec3 sub(pme_vec3 a, pme_vec3 b) { return YMPME_VecSub(a, b); }
static pme_vec3 mul(pme_vec3 a, float t) { return YMPME_VecScale(a, t); }
static float dot(pme_vec3 a, pme_vec3 b) { return YMPME_VecDot(a, b); }
static float length(pme_vec3 a) { return sqrtf(dot(a, a)); }
static pme_vec3 unit(pme_vec3 a) { return mul(a, 1 / length(a)); }
static pme_vec3 position(const demo_scene *s, int b)
{
	ske_mat34 m = s->poseBones[b].global;
	return V(m.m[0][3] * .1f, m.m[1][3] * .1f, m.m[2][3] * .1f);
}
static int worldRotation(demo_scene *s, int b, ske_mat34 world)
{
	ske_mat34 inverse;
	for (int j = 0; j < 3; j++)
		world.m[j][3] = s->poseBones[b].global.m[j][3];
	if (YMSKE_Mat_Inverse(s->poseBones[s->bones[b].parent].global, &inverse) != SKE_OK)
		return 0;
	s->poseBones[b].local = YMSKE_Mat_Multiply(inverse, world);
	return YMSKE_Pose_Update(&s->pose) == SKE_OK;
}
static int aim(demo_scene *s, int b, int child, pme_vec3 target)
{
	pme_vec3 a = unit(sub(position(s, child), position(s, b))),
			 c = unit(sub(target, position(s, b)));
	float cosine = YMPME_Clamp(dot(a, c), -1, 1);
	pme_vec3 axis = YMPME_VecCross(a, c);
	if (cosine < -.9999f)
	{
		axis = YMPME_VecCross(a, V(0, 1, 0));
		if (length(axis) < .01f)
			axis = YMPME_VecCross(a, V(1, 0, 0));
		axis = unit(axis);
	}
	ske_quat q = {axis.x, axis.y, axis.z, cosine < -.9999f ? 0 : 1 + cosine};
	ske_mat34 r;
	if (YMSKE_Mat_TRS((ske_vec3){0, 0, 0}, q, (ske_vec3){1, 1, 1}, &r) != SKE_OK)
		return 0;
	return worldRotation(s, b, YMSKE_Mat_Multiply(r, s->poseBones[b].global));
}

static int leg(demo_scene *h, int hip, int knee, int ankle, pme_vec3 target)
{
	pme_vec3 origin = position(h, hip);
	float a = length(sub(position(h, knee), origin)),
		  b = length(sub(position(h, ankle), position(h, knee)));
	pme_vec3 ray = sub(target, origin);
	float d = YMPME_Clamp(length(ray), fabsf(a - b) + .0005f, a + b - .0005f);
	pme_vec3 along = unit(ray), pole = V(0, 0, 1);
	pole = unit(sub(pole, mul(along, dot(pole, along))));
	float x = (a * a - b * b + d * d) / (2 * d), y = sqrtf(fmaxf(0, a * a - x * x));
	ske_mat34 foot = h->poseBones[ankle].global;
	return aim(h, hip, knee, add(origin, add(mul(along, x), mul(pole, y)))) &&
		   aim(h, knee, ankle, add(origin, mul(along, d))) && worldRotation(h, ankle, foot);
}
const char *HumanGround_Mode(int mode)
{
	static const char *names[] = {"WALK", "STAND / WEIGHT SHIFT", "SQUAT", "LIFT BOTH FEET"};
	return mode >= 0 && mode < 4 ? names[mode] : "UNKNOWN";
}
int HumanGround_Enable(human_skin_rig *s, int mode)
{
	if (mode < 0 || mode > 3)
		return 0;
	human_ground *g = &s->ground;
	s->self.enabled = 0;
	if (!Demo_Update(&s->human, 0))
		return 0;
	if (!g->enabled)
	{
		if (strcmp(s->human.boneNames[15].text, "Feet top .L") ||
			strcmp(s->human.boneNames[61].text, "Feet top .R"))
			return 0;
		g->floorY = 1e9f;
		for (unsigned i = 0; i < s->human.asset.info.vertexCount; i++)
			g->floorY = fminf(g->floorY, s->human.vertices[i].position.y * .1f);
		g->floorY -= .002f;
		g->ankles[0] = position(&s->human, 15);
		g->ankles[1] = position(&s->human, 61);
		for (size_t p = 0; p < s->count; p++)
		{
			float w[2] = {0};
			for (int j = 0; j < 3; j++)
				for (int k = 0; k < YMSKE_MAX_INFLUENCES; k++)
				{
					const ske_weights *weights = &s->human.vertices[s->triangle[p][j]].weights;
					int b = weights->bone[k];
					if (b == 15 || b == 16)
						w[0] += weights->weight[k] / 3;
					if (b == 61 || b == 62)
						w[1] += weights->weight[k] / 3;
				}
			g->foot[p] = w[0] > .5f ? 1 : w[1] > .5f ? 2 : 0;
		}
		g->floor = YMGRE_MeshGener_Box(24, .08f, 24, (GRErgb24){54, 66, 78}, "ground", "");
		if (!g->floor)
			return 0;
		for (int i = 0; i < g->floor->pointNum; i++)
			g->floor->pointList[i].pos.y += g->floorY * 10 - .04f;
		if (!YMGRE_Object_EnableVertexAttributes(g->floor))
			return 0;
		g->floorNode = (gre_listnode){sizeof(*g->floor), g->floor, NULL};
		s->human.objectNode.next = &g->floorNode;
		s->human.objects.len = 2;
	}
	g->enabled = 1;
	g->mode = mode;
	g->clock = 0;
	g->fresh = 1;
	g->travel = 0;
	g->plane = (pme_skin_plane){V(0, 1, 0), V(0, 0, 0), V(0, 0, 0), g->floorY, .006f};
	g->material = (pme_skin_contact_desc){2000000, 6000, .65f, 40000, 30000};
	s->contactView.overview = 3;
	s->uvZoom = 1;
	s->uvCenterU = s->uvCenterV = .5f;
	s->contactView.observationNormalRange = 20;
	s->contactView.observationTangentRange = 10;
	s->gain = 2;
	s->colorMode = 0;
	s->selectedUv = -1;
	s->loadBone = 16;
	s->human.orbit = .55f;
	s->human.elevation = .15f;
	s->human.distance = 23;
	memset(s->self.moments, 0, sizeof s->self.moments);
	return HumanGround_Pose(s);
}
int HumanGround_Pose(human_skin_rig *s)
{
	human_ground *g = &s->ground;
	demo_scene *h = &s->human;
	for (unsigned i = 0; i < h->asset.info.boneCount; i++)
		h->poseBones[i].local = h->animation.samples[i];
	float t = g->clock;
	g->travel = g->mode == 0 ? .08f * t : 0;
	float squat = g->mode == 2 ? .05f * (1 - cosf(t * 2)) * .5f : 0;
	float air = g->mode == 3 ? .12f : 0;
	/* Root is a world-space transform; body weight shift and flexion are prescribed. */
	h->poseBones[0].local.m[0][3] += .06f * sinf(t * 3.14159265f);
	h->poseBones[0].local.m[1][3] += 10 * (-.018f - squat + air);
	h->poseBones[0].local.m[2][3] += 10 * g->travel;
	if (YMSKE_Pose_Update(&h->pose) != SKE_OK)
		return 0;
	for (int side = 0; side < 2; side++)
	{
		float q = t / 2 + side * .5f, cycle = floorf(q), phase = q - cycle;
		float a = YMPME_Clamp((phase - .6f) / .4f, 0, 1);
		float swing = a * a * (3 - 2 * a);
		g->lift[side] = g->mode == 0 ? .07f * sinf(a * 3.14159265f) * sinf(a * 3.14159265f) : 0;
		pme_vec3 target = g->ankles[side];
		target.y += g->lift[side] + air;
		/* Small compression shift lets both feet exchange measured normal load. */
		target.y += (side ? -1 : 1) * .0012f * sinf(t * 3.14159265f);
		if (g->mode == 0)
			target.z += .16f * (cycle + swing - side * .5f);
		if (!leg(h, side ? 59 : 13, side ? 60 : 14, side ? 61 : 15, target))
			return 0;
	}
	/* Relax the source T-pose arms beside the torso, with a small opposing swing. */
	for (int side = 0; side < 2; side++)
	{
		int arm = side ? 41 : 23, elbow = side ? 42 : 24;
		pme_vec3 at = position(h, arm);
		pme_vec3 target = add(
			at, V(side ? -.045f : .045f, -.22f, .04f * (side ? -1 : 1) * sinf(t * 3.14159265f)));
		if (!aim(h, arm, elbow, target))
			return 0;
	}
	if (YMSKE_GRE_Update(&h->binding, &h->pose) != SKE_OK)
		return 0;
	/* Generated floor vertices are centered; update only their object transform. */
	for (int i = 0; i < g->floor->pointNum; i++)
	{
		g->floor->pointList[i].pos.z += g->travel * 10 - g->renderZ;
		g->floor->pointList_wN[i].base = g->floor->pointList[i];
	}
	g->floor->WorldCoordinate = (gre_fvector4d){0, g->floorY * 10 - .04f, g->travel * 10, 1};
	g->renderZ = g->travel * 10;
	return 1;
}
void HumanGround_Readout(human_skin_rig *s)
{
	human_ground *g = &s->ground;
	g->footLoad[0] = g->footLoad[1] = g->otherLoad = 0;
	g->footClearance[0] = g->footClearance[1] = 1e9f;
	g->footCenter[0] = g->footCenter[1] = V(0, 0, 0);
	for (size_t p = 0; p < s->count; p++)
	{
		int side = (int)g->foot[p] - 1;
		if (side < 0)
		{
			g->otherLoad += s->forces[p].y;
			continue;
		}
		g->footLoad[side] += s->forces[p].y;
		g->footCenter[side] = add(g->footCenter[side], mul(s->patches[p].position, s->forces[p].y));
		for (int k = 0; k < 3; k++)
			g->footClearance[side] =
				fminf(g->footClearance[side],
					  s->human.object->pointList[s->triangle[p][k]].pos.y * .1f - g->floorY);
	}
	for (int side = 0; side < 2; side++)
		if (g->footLoad[side] > 0)
			g->footCenter[side] = mul(g->footCenter[side], 1 / g->footLoad[side]);
}
void HumanGround_Draw(human_skin_rig *s)
{
	human_ground *g = &s->ground;
	gre_line3d lines[64];
	int n = 0;
	float base = floorf(g->travel * 10 / 2) * 2, y = g->floorY * 10 + .015f;
	for (int i = -10; i <= 10; i++)
	{
		float v = i * 1.f;
		lines[n++] = (gre_line3d){{v, y, base - 10, 1}, {v, y, base + 10, 1}, {82, 100, 110}, 1};
		lines[n++] = (gre_line3d){{-10, y, base + v, 1}, {10, y, base + v, 1}, {82, 100, 110}, 1};
	}
	YMGRE_Camera_LineList_Rendering(s->human.camera, lines, (uint32)n, 1);
	n = 0;
	for (int side = 0; side < 2; side++)
		if (g->footLoad[side] > 0)
		{
			pme_vec3 p = mul(g->footCenter[side], 10);
			float top = p.y + .6f + fminf(1, g->footLoad[side] / 40);
			GRErgb24 c = side ? (GRErgb24){255, 181, 78} : (GRErgb24){255, 100, 190};
			lines[n++] = (gre_line3d){{p.x, p.y, p.z, 1}, {p.x, top, p.z, 1}, c, 3};
			lines[n++] = (gre_line3d){{p.x, top, p.z, 1}, {p.x - .12f, top - .18f, p.z, 1}, c, 2};
			lines[n++] = (gre_line3d){{p.x, top, p.z, 1}, {p.x + .12f, top - .18f, p.z, 1}, c, 2};
		}
	YMGRE_Camera_LineList_Rendering(s->human.camera, lines, (uint32)n, 0);
}
void HumanGround_Destroy(human_skin_rig *s)
{
	if (s->ground.floor)
		YMGRE_Free_Object(s->ground.floor);
	s->ground.floor = NULL;
	s->human.objectNode.next = NULL;
	if (s->human.objects.len)
		s->human.objects.len = 1;
	s->ground.enabled = 0;
}
