#include "rig.h"
#include "scene_uv_atlas.h"
#include "YMGRE_Creat.h"
#include "YMGRE_Camera.h"
#include "YMGRE_Free.h"
#include "YMGRE_Coordinates_Transform.h"
#include <stdio.h>
#include <string.h>
#define V YMPME_Vec3
static float norm(pme_vec3 p) { return sqrtf(YMPME_VecDot(p, p)); }
static pme_vec3 unit(pme_vec3 p) { return YMPME_VecScale(p, 1 / norm(p)); }
static pme_vec3 point(const human_skin_rig *s, unsigned v)
{
	gre_fvector4d p = s->human.object->pointList[v].pos;
	return V(p.x * .1f, p.y * .1f, p.z * .1f);
}
static pme_vec3 bonePoint(const human_skin_rig *s, unsigned i)
{
	ske_mat34 b = s->human.poseBones[i].global;
	return V(b.m[0][3] * .1f, b.m[1][3] * .1f, b.m[2][3] * .1f);
}
static float boneWeight(const human_skin_rig *s, size_t patch, unsigned bone)
{
	float weight = 0;
	for (int j = 0; j < 3; j++)
		for (int k = 0; k < YMSKE_MAX_INFLUENCES; k++)
		{
			const ske_weights *w = &s->human.vertices[s->triangle[patch][j]].weights;
			if (w->bone[k] == bone)
				weight += w->weight[k] / 3;
		}
	return weight;
}
static int geometry(human_skin_rig *s, int velocity)
{
	for (size_t i = 0; i < s->count; i++)
	{
		pme_vec3 a = point(s, s->triangle[i][0]), b = point(s, s->triangle[i][1]),
				 c = point(s, s->triangle[i][2]);
		pme_vec3 cross = YMPME_VecCross(YMPME_VecSub(b, a), YMPME_VecSub(c, a));
		float length = norm(cross);
		if (!isfinite(length) || length < 1e-12f)
			return 0;
		pme_surface_patch *p = &s->patches[i];
		p->area = length * .5f;
		p->normal = YMPME_VecScale(cross, 1 / length);
		gre_fvector4d n = s->human.object->pointList_wN[s->triangle[i][0]].normal;
		if (YMPME_VecDot(p->normal, V(n.x, n.y, n.z)) < 0)
			p->normal = YMPME_VecScale(p->normal, -1);
		p->position = YMPME_VecScale(YMPME_VecAdd(YMPME_VecAdd(a, b), c), 1.f / 3);
		p->velocity = velocity > 0
						  ? YMPME_VecScale(YMPME_VecSub(p->position, s->previousPoint[i]), 120)
						  : V(0, 0, 0);
		if (velocity >= 0)
			s->previousPoint[i] = p->position;
	}
	return 1;
}
const human_uv_map *HumanSkin_UV(const human_skin_rig *s)
{
	if (s->contactView.overview == 1)
		return &s->contactView.density;
	return s->source ? &s->sourceUv : &s->autoUv;
}
static int localForce(const human_skin_rig *s, size_t patch, pme_vec3 force, pme_vec3 *out)
{
	pme_vec3 p[3];
	for (int j = 0; j < 3; j++)
		p[j] = point(s, s->triangle[patch][j]);
	return DemoForce_Local(p,
						   (s->contactView.overview != 2 ? s->contactView.densityCorners
							: s->source					 ? s->sourceCorners
														 : s->autoCorners) +
							   patch * 6,
						   s->patches[patch].normal, force, out);
}
static int probePosition(human_skin_rig *s, int velocity)
{
	pme_surface_patch *p = &s->patches[s->probePatch];
	s->normal = p->normal;
	/* Probe control always uses bind auto-UV axes; switching the displayed UV cannot alter motion.
	 */
	pme_vec3 tri[3], ex, ey, ez;
	for (int j = 0; j < 3; j++)
		tri[j] = point(s, s->triangle[s->probePatch][j]);
	const float *uv = s->autoCorners + s->probePatch * 6;
	if (!DemoForce_Local(tri, uv, p->normal, V(1, 0, 0), &ex) ||
		!DemoForce_Local(tri, uv, p->normal, V(0, 1, 0), &ey) ||
		!DemoForce_Local(tri, uv, p->normal, V(0, 0, 1), &ez))
		return 0;
	s->tangentU = V(ex.x, ey.x, ez.x);
	s->tangentV = V(ex.y, ey.y, ez.y);
	float u = s->offsetU + (s->rub ? .006f * sinf((float)s->time * 4) : 0),
		  v = s->offsetV + (s->rub ? .004f * (1 - cosf((float)s->time * 4)) : 0);
	pme_vec3 center =
		YMPME_VecAdd(p->position, YMPME_VecScale(p->normal, s->probe.radius - s->indentation));
	center = YMPME_VecAdd(
		center, YMPME_VecAdd(YMPME_VecScale(s->tangentU, u), YMPME_VecScale(s->tangentV, v)));
	s->probe.center = center;
	s->probe.velocity =
		velocity ? YMPME_VecScale(YMPME_VecSub(center, s->previousProbe), 120) : V(0, 0, 0);
	s->previousProbe = center;
	return 1;
}
int HumanSkin_Select(human_skin_rig *s, int patch)
{
	if (patch < 0 || (size_t)patch >= s->count)
		return 0;
	s->probePatch = patch;
	s->offsetU = s->offsetV = 0;
	s->selectedUv = patch;
	s->loadBone = 11;
	float best = -1;
	for (unsigned b = 11; b < s->human.asset.info.boneCount; b++)
	{
		float w = boneWeight(s, (size_t)patch, b);
		if (w > best)
		{
			best = w;
			s->loadBone = (int)b;
		}
	}
	return probePosition(s, 0);
}
static const int regionBone[] = {19, 21, 23, 41, 25, 13, 59, 14, 19};
static const char *regionName[] = {"CHEST",	  "HEAD",	 "ARM L",  "ARM R", "WRIST L",
								   "THIGH L", "THIGH R", "SHIN L", "BACK"};
const char *HumanSkin_RegionName(int r) { return r >= 0 && r < 9 ? regionName[r] : "CUSTOM"; }
int HumanSkin_Region(human_skin_rig *s, int region)
{
	if (region < 0 || region > 8)
		return 0;
	s->human.selected = regionBone[region];
	s->angle = 0;
	if (!Demo_Update(&s->human, 0) || !geometry(s, 0))
		return 0;
	unsigned bone = (unsigned)regionBone[region];
	pme_vec3 center = V(0, 0, 0);
	float total = 0;
	for (size_t i = 0; i < s->count; i++)
	{
		float weight = boneWeight(s, i, bone) * s->patches[i].area;
		center = YMPME_VecAdd(center, YMPME_VecScale(s->patches[i].position, weight));
		total += weight;
	}
	if (total <= 0)
		return 0;
	center = YMPME_VecScale(center, 1 / total);
	int pick = -1;
	float best = 1e30f;
	for (size_t i = 0; i < s->count; i++)
	{
		if (boneWeight(s, i, bone) < .15f || s->patches[i].normal.z * (region == 8 ? -1 : 1) < .25f)
			continue;
		pme_vec3 delta = YMPME_VecSub(s->patches[i].position, center);
		float score = YMPME_VecDot(delta, delta);
		if (score < best)
		{
			best = score;
			pick = (int)i;
		}
	}
	if (pick < 0)
		return 0;
	s->region = region;
	s->human.orbit = region == 8 ? 3.14159f : .2f;
	return HumanSkin_Select(s, pick);
}
int HumanSkin_Joint(human_skin_rig *s, int bone)
{
	if (bone < 11 || bone >= (int)s->human.asset.info.boneCount)
		return 0;
	s->human.selected = bone;
	s->angle = 0;
	return Demo_Update(&s->human, 0) && geometry(s, 0) && probePosition(s, 0);
}
int HumanSkin_Init(human_skin_rig *s)
{
	memset(s, 0, sizeof *s);
	s->probePatch = -1;
	s->selectedUv = -1;
	if (!Demo_Init(&s->human))
		return 0;
	float positions[DEMO_POINTS * 3], extent = 0;
	unsigned indices[DEMO_FACES * 3];
	int charts[DEMO_FACES * 3];
	char error[256] = {0};
	for (unsigned v = 0; v < s->human.asset.info.vertexCount; v++)
	{
		gre_fvector4d p = s->human.object->pointList[v].pos;
		positions[v * 3] = p.x;
		positions[v * 3 + 1] = p.y;
		positions[v * 3 + 2] = p.z;
		extent = fmaxf(extent, fmaxf(fabsf(p.x), fmaxf(fabsf(p.y), fabsf(p.z))));
	}
	if (extent <= 0)
		return 0;
	for (unsigned i = 0; i < s->human.asset.info.vertexCount * 3; i++)
		positions[i] = roundf(positions[i] / extent * 100000) / 100000;
	for (int f = 0; f < s->human.object->polygonNum; f++)
	{
		GRE_Polygon4d face = &s->human.object->polygonList[f];
		if (face->num != 3)
			return 0;
		for (int j = 0; j < 3; j++)
			if (face->index[j] >= s->human.asset.info.vertexCount)
				return 0;
		pme_vec3 a = point(s, face->index[0]), b = point(s, face->index[1]),
				 c = point(s, face->index[2]);
		if (norm(YMPME_VecCross(YMPME_VecSub(b, a), YMPME_VecSub(c, a))) < 1e-12f)
			continue;
		size_t at = s->count++;
		s->face[at] = (unsigned)f;
		for (int j = 0; j < 3; j++)
		{
			unsigned v = face->index[j];
			s->triangle[at][j] = (uint16_t)v;
			indices[at * 3 + j] = v;
			s->sourceCorners[at * 6 + j * 2] = s->human.object->pointList[v].u;
			s->sourceCorners[at * 6 + j * 2 + 1] = s->human.object->pointList[v].v;
		}
	}
	SceneUvAtlasMesh input = {
		positions,		indices, s->human.asset.info.vertexCount, (unsigned)s->count * 3,
		s->autoCorners, charts};
	if (!SceneUv_AtlasUnwrap(&input, 1, 1, &s->charts, error, sizeof error))
	{
		fprintf(stderr, "Human UV: %s\n", error);
		return 0;
	}
	if (!HumanUV_Init(&s->autoUv, s->autoCorners, s->count) ||
		!HumanUV_Init(&s->sourceUv, s->sourceCorners, s->count))
		return 0;
	s->uvCamera = YMGRE_Creat_Camera(7, UV_VIEW, UV_VIEW, 28, 28, 28, 28);
	s->legend = YMGRE_Creat_Camera(8, UV_VIEW, 16, 28, 28, 28, 28);
	if (!s->uvCamera || !s->legend)
		return 0;
	s->white = (GRErgb24){255, 255, 255};
	s->heatMaterial.name = s->human.object->materiaName;
	s->heatMaterial.nameLen = (uint8)(strlen(s->heatMaterial.name) + 1);
	s->heatMaterial.valid = 1;
	s->heatMaterial.unlit = 0;
	s->heatMaterial.diffuse = s->white;
	s->heatMaterial.ambient = s->white;
	s->human.distance = 20;
	s->heatMaterial.pixel = &s->white;
	s->heatMaterial.width = s->heatMaterial.height = 1;
	s->heatNode = (gre_listnode){sizeof s->heatMaterial, &s->heatMaterial, NULL};
	s->heat = 1;
	s->gain = 2;
	s->uvZoom = 1;
	s->uvCenterU = s->uvCenterV = .5f;
	s->indentation = .005f;
	s->probe.radius = .075f;
	s->contact = (pme_skin_contact_desc){200000, 500, .6f, 3000, 6000};
	s->touch.cells = s->taxels;
	s->touch.capacity = DEMO_FACES;
	pme_force_device_desc fd = YMPME_ForceDevice_DefaultDesc();
	fd.force.cutoffHz = 15;
	fd.force.resolution = .001f;
	if (YMPME_ForceDevice_Init(&s->loadCell, &fd, 123) != PME_OK)
		return 0;
	if (!geometry(s, 0) || !HumanSkin_Region(s, 0) || !HumanSelf_Init(s) ||
		!HumanContactView_Init(s))
		return 0;
	printf("Human: %u vertices, %zu patches, %u bones, %d weight slots, GRE UV %d islands / %zu "
		   "overlap / %zu subpixel\n",
		   s->human.asset.info.vertexCount, s->count, s->human.asset.info.boneCount,
		   YMSKE_MAX_INFLUENCES, s->charts, s->autoUv.overlapPixels, s->autoUv.uncoveredPatches);
	return 1;
}
int HumanSkin_Step(human_skin_rig *s)
{
	const float dt = 1.f / 120;
	s->time += dt;
	if (s->ground.enabled)
	{
		s->ground.clock += dt;
		if (!HumanGround_Pose(s) || !geometry(s, s->ground.fresh ? 0 : 1))
			return 0;
		s->ground.fresh = 0;
		s->probe.center = bonePoint(s, 11);
		s->ground.plane.reference = s->probe.center;
		if (YMPME_SkinContact_Plane(&s->ground.material, s->patches, s->count, &s->ground.plane,
									s->forces, DEMO_FACES, &s->reaction) != PME_OK)
			return 0;
		HumanGround_Readout(s);
	}
	else if (s->self.enabled)
	{
		s->self.clock += dt;
		pme_vec3 previousDesired = s->self.desired;
		pme_vec3 palm = HumanSelf_Target(s);
		if (!s->self.filtered && s->self.correction > 0 &&
			norm(YMPME_VecSub(palm, previousDesired)) < .005f)
			palm.z = fmaxf(palm.z, s->self.corrected.z - .003f);
		s->self.correction = palm.z - s->self.desired.z;
		/* Re-evaluate the skinned geometry after each IK correction; intermediate queries
		 * never overwrite previous physical-step positions used for velocity. */
		for (int it = 0; it < 8; it++)
		{
			if (!HumanSelf_Pose(s, palm) || !geometry(s, -1) || !HumanSelf_Contact(s))
				return 0;
			float excess = s->self.result.maxDepth - .004f;
			if (excess <= .0001f || s->self.filtered)
				break;
			float correction = fminf(.03f, excess);
			palm.z += correction;
			s->self.correction += correction;
		}
		s->probe.center = bonePoint(s, 25);
		if (!geometry(s, 1) || !HumanSelf_Contact(s))
			return 0;
		s->reaction.probeWrench = s->self.result.sourceWrench;
		s->reaction.contacts = s->self.result.contacts;
		s->reaction.maxPenetration = s->self.result.maxDepth;
		s->reaction.saturated = s->self.result.saturated;
	}
	else
	{
		float angle = s->angle + (s->motion ? 20 * sinf((float)s->time * 1.5f) : 0);
		if (!Demo_Update(&s->human, angle) || !geometry(s, 1) || !probePosition(s, 1))
			return 0;
		if (YMPME_SkinContact_Sphere(&s->contact, s->patches, s->count, &s->probe, s->forces,
									 DEMO_FACES, &s->reaction) != PME_OK)
			return 0;
		memset(s->self.moments, 0, sizeof s->self.moments);
	}
	pme_sensor_stamp stamp = {s->time, s->time, (uint64_t)llround(s->time / dt), 1};
	if (YMPME_SurfaceTactile_Measure(s->patches, s->forces, s->count, &stamp, s->probe.center, 1,
									 &s->touch) != PME_OK)
		return 0;
	if (s->self.enabled)
		for (size_t i = 0; i < s->count; i++)
			s->touch.wrench.torque = YMPME_VecAdd(s->touch.wrench.torque, s->self.moments[i]);
	memset(s->boneLoads, 0, sizeof s->boneLoads);
	s->totalNormal = s->peakPressure = 0;
	s->self.handNormal = s->self.bodyNormal = 0;
	for (size_t p = 0; p < s->count; p++)
	{
		s->totalNormal += s->taxels[p].normalForce;
		if (s->self.enabled)
		{
			if (s->self.faces[p].region == 1)
				s->self.handNormal += s->taxels[p].normalForce;
			if (s->self.faces[p].region == 2)
				s->self.bodyNormal += s->taxels[p].normalForce;
		}
		s->peakPressure = fmaxf(s->peakPressure, s->taxels[p].pressure);
		if (YMPME_VecDot(s->forces[p], s->forces[p]) == 0)
			continue;
		for (int j = 0; j < 3; j++)
			for (int k = 0; k < YMSKE_MAX_INFLUENCES; k++)
			{
				const ske_weights *w = &s->human.vertices[s->triangle[p][j]].weights;
				float weight = w->weight[k] / 3;
				if (weight <= 0)
					continue;
				unsigned b = w->bone[k];
				pme_vec3 f = YMPME_VecScale(s->forces[p], weight);
				s->boneLoads[b].force = YMPME_VecAdd(s->boneLoads[b].force, f);
				s->boneLoads[b].torque = YMPME_VecAdd(
					s->boneLoads[b].torque,
					YMPME_VecAdd(
						YMPME_VecCross(YMPME_VecSub(s->patches[p].position, bonePoint(s, b)), f),
						YMPME_VecScale(s->self.moments[p], weight)));
			}
	}
	if (s->self.enabled)
	{
		s->loadBone = 25;
		for (int b = 26; b <= 39; b++)
			if (norm(s->boneLoads[b].force) > norm(s->boneLoads[s->loadBone].force))
				s->loadBone = b;
	}
	pme_feedback_state state = {0};
	state.valid = PME_FEEDBACK_POSE;
	state.time = s->time;
	state.position = s->probe.center;
	state.rotation = YMPME_Rotation_Identity();
	pme_force_sample input = {stamp, s->reaction.probeWrench, 1};
	return YMPME_ForceDevice_Measure(&s->loadCell, &input, &state, &s->loadReading) == PME_OK &&
		   Demo_Validate(&s->human);
}
static GRErgb24 heat(float value, float maximum)
{
	float t = YMPME_Clamp(value / maximum, 0, 1);
	return (GRErgb24){(uint8)(35 + 220 * t), (uint8)(100 + 110 * (1 - fabsf(2 * t - 1))),
					  (uint8)(170 * (1 - t) + 30)};
}
/* Display-only overlay: read animated vertices each frame, including hidden faces.
 * Lines use GRE clipping; no force, material, vertex colour or observation is changed. */
static void drawPickTest(const human_skin_rig *s)
{
	if (!s->pickTest || s->selectedUv < 0 || (size_t)s->selectedUv >= s->count)
		return;
	gre_fvector4d v[3];
	for (int j = 0; j < 3; j++)
		v[j] = s->human.object->pointList[s->triangle[s->selectedUv][j]].pos;
	gre_line3d lines[27];
	for (int j = 0; j < 24; j++)
	{
		float t = (j + 1) / 25.f;
		gre_fvector4d a = {v[0].x + t * (v[1].x - v[0].x), v[0].y + t * (v[1].y - v[0].y),
						   v[0].z + t * (v[1].z - v[0].z), 1};
		gre_fvector4d b = {v[0].x + t * (v[2].x - v[0].x), v[0].y + t * (v[2].y - v[0].y),
						   v[0].z + t * (v[2].z - v[0].z), 1};
		lines[j] = (gre_line3d){a, b, {255, 210, 0}, 1};
	}
	for (int j = 0; j < 3; j++)
		lines[24 + j] = (gre_line3d){v[j], v[(j + 1) % 3], {255, 255, 80}, 3};
	YMGRE_Camera_LineList_Rendering(s->human.camera, lines, 27, 0);
}

int HumanSkin_IsContactMarked(const human_skin_rig *s, size_t face)
{
	return s->contactMarks && s->self.enabled && face < s->count &&
		   YMPME_VecDot(s->forces[face], s->forces[face]) > 1e-12f;
}
static void drawContactMarks(const human_skin_rig *s)
{
	/* X-ray outlines expose the contacting palm, which faces into the torso.
	 * Brightness is a contact indicator only; RGB values remain linear force readings. */
	gre_line3d lines[96];
	unsigned n = 0;
	for (size_t p = 0; p < s->count; p++)
		if (HumanSkin_IsContactMarked(s, p))
		{
			for (int j = 0; j < 3; j++)
				lines[n++] =
					(gre_line3d){s->human.object->pointList[s->triangle[p][j]].pos,
								 s->human.object->pointList[s->triangle[p][(j + 1) % 3]].pos,
								 {245, 245, 245},
								 1};
			if (n == 96)
			{
				YMGRE_Camera_LineList_Rendering(s->human.camera, lines, n, 0);
				n = 0;
			}
		}
	if (n)
		YMGRE_Camera_LineList_Rendering(s->human.camera, lines, n, 0);
}

void HumanSkin_Render(human_skin_rig *s)
{
	pme_vec3 sums[DEMO_POINTS] = {{0}};
	float areas[DEMO_POINTS] = {0};
	s->rgbClipped = 0;
	for (size_t p = 0; p < s->count; p++)
	{
		pme_vec3 local = V(0, 0, 0);
		localForce(s, p, s->forces[p], &local);
		s->localForces[p] = local;
		int clipped = 0;
		s->colors[p] = s->colorMode == 0
						   ? DemoForce_RGB(local, 2 * s->gain, s->gain, &clipped)
						   : heat(s->colorMode == 1 ? norm(s->forces[p]) : s->taxels[p].pressure,
								  s->colorMode == 1 ? 2 * s->gain : 2000 * s->gain);
		s->rgbClipped += clipped;
		for (int j = 0; j < 3; j++)
		{
			unsigned v = s->triangle[p][j];
			pme_vec3 value =
				s->colorMode == 0
					? local
					: V(s->colorMode == 1 ? norm(s->forces[p]) : s->taxels[p].pressure, 0, 0);
			sums[v] = YMPME_VecAdd(sums[v], YMPME_VecScale(value, s->patches[p].area));
			areas[v] += s->patches[p].area;
		}
	}
	for (unsigned v = 0; v < s->human.asset.info.vertexCount; v++)
	{
		pme_vec3 value = areas[v] > 0 ? YMPME_VecScale(sums[v], 1 / areas[v]) : V(0, 0, 0);
		s->human.object->pointList_wN[v].color =
			!s->heat			? s->white
			: s->colorMode == 0 ? DemoForce_RGB(value, 2 * s->gain, s->gain, NULL)
								: heat(value.x, s->colorMode == 1 ? 2 * s->gain : 2000 * s->gain);
	}
	gre_list materials = s->human.materials;
	if (s->heat)
	{
		s->heatNode.next = materials.listhead;
		s->human.materials = (gre_list){&s->heatNode, materials.len + 1};
	}
	if (!s->self.enabled && !s->ground.enabled)
		Demo_Render(&s->human, s->wire, s->bones);
	else
	{
		demo_scene *h = &s->human;
		float r = h->distance * cosf(h->elevation);
		gre_fvector4d eye = {r * sinf(h->orbit), 2 + h->distance * sinf(h->elevation),
							 r * cosf(h->orbit), 1},
					  at = {0, 2, 0, 1};
		if (s->ground.enabled)
		{
			at.y = -.25f;
			at.z = s->ground.travel * 10;
			eye.y = at.y + h->distance * sinf(h->elevation);
			eye.z += at.z;
		}
		YMGRE_UVNCamera_PositionInit(h->camera, &eye, &at, NULL, 0);
		h->camera->wireFrame = GRE_Render_Solid;
		h->object->wireFrame = (uint8)(s->wire != 0);
		YMGRE_Camera_TanglePipline_wN(h->camera, &h->lights, &h->objects, &h->materials,
									  &h->workspace);
		GRE_RenderTarget target = YMGRE_Camera_GetRenderTarget(h->camera);
		for (size_t i = 0; i < DEMO_WIDTH * DEMO_HEIGHT; i++)
			if (target->zbuff[i] >= h->camera->frustum.Zfar)
				target->data[i] = GRE_FramePixel_From_RGB24((GRErgb24){15, 21, 31});
		if (s->bones)
		{
			gre_line3d boneLines[DEMO_BONES];
			int n = 0;
			for (unsigned b = 11; b < h->asset.info.boneCount; b++)
			{
				ske_vec3 a = YMSKE_Mat_Point(h->poseBones[b].global, (ske_vec3){0, 0, 0});
				ske_vec3 z = YMSKE_Mat_Point(h->poseBones[b].global, h->asset.tails[b]);
				boneLines[n++] =
					(gre_line3d){{a.x, a.y, a.z, 1}, {z.x, z.y, z.z, 1}, {90, 220, 255}, 1};
			}
			YMGRE_Camera_LineList_Rendering(h->camera, boneLines, (uint32)n, 0);
		}
	}
	s->human.materials = materials;
	if (s->contactView.overview == 3)
		HumanContactView_GridRender(s, YMGRE_Camera_GetRenderTarget(s->uvCamera));
	else
		HumanUV_Render(HumanSkin_UV(s), YMGRE_Camera_GetRenderTarget(s->uvCamera), s->colors,
					   s->selectedUv, s->uvCenterU, s->uvCenterV, s->uvZoom);
	if (s->contactView.overview == 1)
		HumanContactView_Edges(s, YMGRE_Camera_GetRenderTarget(s->uvCamera), 0, s->uvCenterU,
							   s->uvCenterV, s->uvZoom);
	GRE_RenderTarget legend = YMGRE_Camera_GetRenderTarget(s->legend);
	for (int x = 0; x < UV_VIEW; x++)
	{
		GRErgb24 c;
		if (s->colorMode == 0)
		{
			int part = x * 3 / UV_VIEW, start = part * UV_VIEW / 3, end = (part + 1) * UV_VIEW / 3;
			float t = (float)(x - start) / (end - start - 1);
			c = DemoForce_RGB(part == 0	  ? V(0, 0, 2 * s->gain * t)
							  : part == 1 ? V(s->gain * (2 * t - 1), 0, 0)
										  : V(0, s->gain * (2 * t - 1), 0),
							  2 * s->gain, s->gain, NULL);
		}
		else
			c = heat((float)x, UV_VIEW - 1);
		for (int y = 0; y < 16; y++)
			legend->data[y * UV_VIEW + x] = GRE_FramePixel_From_RGB24(c);
	}
	gre_line3d lines[96 + 3];
	int count = 0;
	for (int axis = 0; axis < (s->self.enabled || s->ground.enabled ? 0 : 3); axis++)
		for (int j = 0; j < 32; j++)
		{
			float a = j * 6.2831853f / 32, b = (j + 1) * 6.2831853f / 32;
			pme_vec3 t = axis == 0	 ? s->tangentU
						 : axis == 1 ? s->tangentV
									 : s->normal,
					 u = axis == 0	 ? s->tangentV
						 : axis == 1 ? s->normal
									 : s->tangentU;
			pme_vec3 pa =
				YMPME_VecScale(YMPME_VecAdd(s->probe.center,
											YMPME_VecScale(YMPME_VecAdd(YMPME_VecScale(t, cosf(a)),
																		YMPME_VecScale(u, sinf(a))),
														   s->probe.radius)),
							   10);
			pme_vec3 pb =
				YMPME_VecScale(YMPME_VecAdd(s->probe.center,
											YMPME_VecScale(YMPME_VecAdd(YMPME_VecScale(t, cosf(b)),
																		YMPME_VecScale(u, sinf(b))),
														   s->probe.radius)),
							   10);
			lines[count++] =
				(gre_line3d){{pa.x, pa.y, pa.z, 1}, {pb.x, pb.y, pb.z, 1}, {245, 220, 125}, 1};
		}
	if (s->arrows)
	{
		int selected = s->probePatch;
		if (s->self.enabled)
		{
			if (s->selectedUv >= 0)
				selected = s->selectedUv;
			else
				for (size_t p = 0; p < s->count; p++)
					if (norm(s->forces[p]) > norm(s->forces[selected]))
						selected = (int)p;
		}
		pme_vec3 end = YMPME_VecScale(s->patches[selected].position, 10), f = s->forces[selected];
		if (norm(f) > 1e-6f)
		{
			pme_vec3 dir = unit(f), start = YMPME_VecSub(end, YMPME_VecScale(dir, .7f)),
					 head = YMPME_VecSub(end, YMPME_VecScale(dir, .15f));
			pme_vec3 side = YMPME_VecScale(s->tangentU, .12f), a = YMPME_VecAdd(head, side),
					 b = YMPME_VecSub(head, side);
			lines[count++] = (gre_line3d){
				{start.x, start.y, start.z, 1}, {end.x, end.y, end.z, 1}, {255, 255, 210}, 2};
			lines[count++] =
				(gre_line3d){{end.x, end.y, end.z, 1}, {a.x, a.y, a.z, 1}, {255, 255, 210}, 2};
			lines[count++] =
				(gre_line3d){{end.x, end.y, end.z, 1}, {b.x, b.y, b.z, 1}, {255, 255, 210}, 2};
		}
	}
	YMGRE_Camera_LineList_Rendering(s->human.camera, lines, (uint32)count, 0);
	if (s->ground.enabled)
		HumanGround_Draw(s);
	drawContactMarks(s);
	drawPickTest(s);
}

int HumanSkin_PickUV(const human_skin_rig *s, float x, float y)
{
	if (x < 0 || y < 0 || x >= UV_VIEW || y >= UV_VIEW)
		return -1;
	if (s->contactView.overview == 3)
		return HumanContactView_GridPick(s, s->uvCenterU + ((x + .5f) / UV_VIEW - .5f) / s->uvZoom,
										 s->uvCenterV + ((y + .5f) / UV_VIEW - .5f) / s->uvZoom);
	int px = (int)((s->uvCenterU + ((x + .5f) / UV_VIEW - .5f) / s->uvZoom) * UV_SIZE);
	int py = (int)((s->uvCenterV + ((y + .5f) / UV_VIEW - .5f) / s->uvZoom) * UV_SIZE);
	return HumanUV_Pick(HumanSkin_UV(s), px, py);
}
void HumanSkin_ZoomUV(human_skin_rig *s, float x, float y, float factor)
{
	if (!isfinite(x + y + factor) || factor <= 0)
		return;
	float u = s->uvCenterU + ((x + .5f) / UV_VIEW - .5f) / s->uvZoom,
		  v = s->uvCenterV + ((y + .5f) / UV_VIEW - .5f) / s->uvZoom;
	s->uvZoom = YMPME_Clamp(s->uvZoom * factor, 1, 8);
	float half = .5f / s->uvZoom;
	s->uvCenterU = YMPME_Clamp(u - ((x + .5f) / UV_VIEW - .5f) / s->uvZoom, half, 1 - half);
	s->uvCenterV = YMPME_Clamp(v - ((y + .5f) / UV_VIEW - .5f) / s->uvZoom, half, 1 - half);
}
int HumanSkin_Pick3D(const human_skin_rig *s, float x, float y)
{
	float best = 1e30f;
	int pick = -1;
	for (size_t p = 0; p < s->count; p++)
	{
		gre_fvector4d v[3];
		int ok = 1;
		for (int j = 0; j < 3; j++)
		{
			gre_fvector4d world = s->human.object->pointList[s->triangle[p][j]].pos;
			YMGRE_Point_WorldToCamera(&world, &v[j], &s->human.camera->move.TMat);
			if (v[j].z <= s->human.camera->frustum.Znear)
			{
				ok = 0;
				break;
			}
			YMGRE_Point_CameraToViewPlane(&v[j], s->human.camera->perspectPlane.Dis);
			YMGRE_Point_ViewPlaneToWindows(&v[j], s->human.camera);
		}
		if (!ok)
			continue;
		float det = (v[1].y - v[2].y) * (v[0].x - v[2].x) + (v[2].x - v[1].x) * (v[0].y - v[2].y);
		if (fabsf(det) < 1e-8f)
			continue;
		float a = ((v[1].y - v[2].y) * (x - v[2].x) + (v[2].x - v[1].x) * (y - v[2].y)) / det;
		float b = ((v[2].y - v[0].y) * (x - v[2].x) + (v[0].x - v[2].x) * (y - v[2].y)) / det,
			  c = 1 - a - b;
		if (a < 0 || b < 0 || c < 0)
			continue;
		float z = 1 / (a / v[0].z + b / v[1].z + c / v[2].z);
		if (z < best)
		{
			best = z;
			pick = (int)p;
		}
	}
	return pick;
}
void HumanSkin_Destroy(human_skin_rig *s)
{
	HumanGround_Destroy(s);
	if (s->uvCamera)
		YMGRE_Free_Camera(s->uvCamera);
	if (s->legend)
		YMGRE_Free_Camera(s->legend);
	s->uvCamera = s->legend = NULL;
	Demo_Destroy(&s->human);
}
