#include "rig.h"
#include <string.h>
#include "YMGRE_Creat.h"
#include "YMGRE_Free.h"
#define V YMPME_Vec3
static pme_vec3 point(const hand_skin_rig *s, unsigned i)
{
	gre_fvector4d p = s->hand.object->pointList[i].pos;
	return V(p.x * .1f, p.y * .1f, p.z * .1f);
}
static int geometry(hand_skin_rig *s)
{
	for (size_t i = 0; i < s->count; ++i)
	{
		const uint16_t *t = handTriangles[s->faces[i]];
		pme_vec3 a = point(s, t[0]), b = point(s, t[1]), c = point(s, t[2]);
		pme_vec3 cross = YMPME_VecCross(YMPME_VecSub(b, a), YMPME_VecSub(c, a));
		float length = sqrtf(YMPME_VecDot(cross, cross));
		if (!isfinite(length) || length < 1e-12f)
			return 0;
		pme_surface_patch *p = &s->patches[i];
		p->area = length * .5f;
		p->position = YMPME_VecScale(YMPME_VecAdd(YMPME_VecAdd(a, b), c), 1.f / 3);
		p->normal = YMPME_VecScale(cross, 1 / length);
		gre_fvector4d n = s->hand.object->pointList_wN[t[0]].normal;
		if (YMPME_VecDot(p->normal, V(n.x, n.y, n.z)) < 0)
			p->normal = YMPME_VecScale(p->normal, -1);
		p->velocity = V(0, 0, 0);
		for (int k = 0; k < HAND_BONES; ++k)
			if (s->weights[i][k] > 0)
			{
				const pme_rigid_body *body = &s->hand.bodies[k];
				pme_vec3 v = YMPME_VecAdd(
					body->velocity, YMPME_VecCross(body->angularVelocity,
												   YMPME_VecSub(p->position, body->position)));
				p->velocity = YMPME_VecAdd(p->velocity, YMPME_VecScale(v, s->weights[i][k]));
			}
	}
	return 1;
}
int HandSkin_Select(hand_skin_rig *s, int region)
{
	if (region < 0 || region > 5)
		return 0;
	/* 每次选择恢复张手以建立稳定世界按压位置。之后球不会跟随皮肤逃逸。 */
	if (!Demo_Reset(&s->hand) || !Demo_Update(&s->hand, 0) || !geometry(s))
		return 0;
	int bone = region ? region * 3 : 0;
	float best = -1e30f;
	size_t pick = SIZE_MAX;
	pme_vec3 target = region ? s->hand.bodies[bone].position : V(0, .06f, .01f);
	for (size_t i = 0; i < s->count; ++i)
	{
		pme_surface_patch *p = &s->patches[i];
		if (s->weights[i][bone] < .5f || p->normal.z < .5f)
			continue;
		pme_vec3 d = YMPME_VecSub(p->position, target);
		float score = -YMPME_VecDot(d, d) + .00003f * p->normal.z;
		if (score > best)
		{
			best = score;
			pick = i;
		}
	}
	if (pick == SIZE_MAX)
		return 0;
	s->anchor = s->patches[pick].position;
	s->normal = s->patches[pick].normal;
	s->region = region;
	s->probe.center =
		YMPME_VecAdd(s->anchor, YMPME_VecScale(s->normal, s->probe.radius - s->indentation));
	s->previousCenter = s->probe.center;
	return 1;
}
int HandSkin_Init(hand_skin_rig *s)
{
	memset(s, 0, sizeof(*s));
	if (!Demo_Init(&s->hand))
		return 0;
	s->hand.orbit = 0;
	s->hand.elevation = 0;
	s->hand.distance = 3.8f;
	s->hand.selectedJoint = -1;
	s->heat = 1;
	s->heatMaterial.name = s->hand.object->materiaName;
	s->heatMaterial.nameLen = (uint8)(strlen(s->heatMaterial.name) + 1);
	s->heatMaterial.valid = 1;
	s->heatMaterial.diffuse = (GRErgb24){255, 255, 255};
	s->heatMaterial.unlit = 1;
	s->heatWhite = (GRErgb24){255, 255, 255};
	s->heatMaterial.pixel = &s->heatWhite;
	s->heatMaterial.width = s->heatMaterial.height = 1;
	s->heatNode = (gre_listnode){sizeof(s->heatMaterial), &s->heatMaterial, NULL};
	s->contact = YMPME_SkinContact_DefaultDesc();
	s->probe.radius = .015f;
	s->indentation = .003f;
	for (unsigned i = 0; i < HAND_FACES; ++i)
	{
		const uint16_t *t = handTriangles[i];
		pme_vec3 cross = YMPME_VecCross(YMPME_VecSub(point(s, t[1]), point(s, t[0])),
										YMPME_VecSub(point(s, t[2]), point(s, t[0])));
		if (YMPME_VecDot(cross, cross) < 1e-24f)
			continue;
		size_t at = s->count++;
		s->faces[at] = i;
		for (int j = 0; j < 3; ++j)
			for (int k = 0; k < YMSKE_MAX_INFLUENCES; ++k)
			{
				float weight = handVertices[t[j]].weights.weight[k];
				if (weight > 0)
					s->weights[at][handVertices[t[j]].weights.bone[k]] += weight / 3;
			}
	}
	s->uvSelected = -1;
	s->uvCamera = YMGRE_Creat_Camera(7, HAND_UV_SIZE, HAND_UV_SIZE, 28, 28, 28, 28);
	s->uvLegend = YMGRE_Creat_Camera(8, HAND_UV_SIZE, 16, 28, 28, 28, 28);
	if (!s->uvCamera || !s->uvLegend || !HandUV_Init(&s->uv, s->faces, s->count))
		return 0;
	s->touch.cells = s->taxels;
	s->touch.capacity = HAND_FACES;
	pme_force_device_desc desc = YMPME_ForceDevice_DefaultDesc();
	desc.force.cutoffHz = 20;
	desc.force.resolution = .0001f;
	desc.torque.resolution = .00001f;
	if (YMPME_ForceDevice_Init(&s->loadCell, &desc, 734) != PME_OK)
		return 0;
	return HandSkin_Select(s, 0);
}
static int step(hand_skin_rig *s, const pme_skin_probe *external, pme_vec3 omega, pme_vec3 gravity,
				float effort)
{
	const float dt = 1.f / 240;
	if (!geometry(s))
		return 0;
	if (external)
	{
		s->probe = *external;
		if (YMPME_SkinContact_SphereRigid(&s->contact, s->patches, s->count, &s->probe, omega,
										  s->forces, HAND_FACES, &s->reaction) != PME_OK)
			return 0;
	}
	else
	{
		s->probe.center =
			YMPME_VecAdd(s->anchor, YMPME_VecScale(s->normal, s->probe.radius - s->indentation));
		s->probe.velocity =
			YMPME_VecScale(YMPME_VecSub(s->probe.center, s->previousCenter), 1 / dt);
		s->previousCenter = s->probe.center;
		if (YMPME_SkinContact_Sphere(&s->contact, s->patches, s->count, &s->probe, s->forces,
									 HAND_FACES, &s->reaction) != PME_OK)
			return 0;
	}
	s->time += dt;
	pme_sensor_stamp stamp = {s->time, s->time, (uint64_t)llround(s->time / dt), 1};
	if (YMPME_SurfaceTactile_Measure(s->patches, s->forces, s->count, &stamp, s->probe.center, 1,
									 &s->touch) != PME_OK)
		return 0;
	pme_feedback_state state = {0};
	state.valid = PME_FEEDBACK_POSE;
	state.time = s->time;
	state.position = s->probe.center;
	state.rotation = YMPME_Rotation_Identity();
	pme_force_sample input = {stamp, s->reaction.probeWrench, 1};
	s->loadCell.desc.forceNoise = s->noise ? .002f : 0;
	s->loadCell.desc.torqueNoise = s->noise ? .00002f : 0;
	if (YMPME_ForceDevice_Measure(&s->loadCell, &input, &state, &s->loadReading) != PME_OK)
		return 0;
	s->normalLoad = YMPME_VecDot(s->loadReading.sample.wrench.force, s->normal);
	s->peakPressure = 0;
	memset(s->vertexPressure, 0, sizeof(s->vertexPressure));
	memset(s->vertexArea, 0, sizeof(s->vertexArea));
	memset(s->vertexForce, 0, sizeof(s->vertexForce));
	for (size_t i = 0; i < s->count; ++i)
	{
		s->peakPressure = fmaxf(s->peakPressure, s->taxels[i].pressure);
		const uint16_t *t = handTriangles[s->faces[i]];
		for (int k = 0; k < 3; ++k)
		{
			s->vertexPressure[t[k]] += s->taxels[i].pressure * s->patches[i].area;
			s->vertexArea[t[k]] += s->patches[i].area;
			s->vertexForce[t[k]] +=
				sqrtf(YMPME_VecDot(s->forces[i], s->forces[i])) * s->patches[i].area;
		}
		/* 应用级权重映射：每份力都作用于同一世界接触点，保留总力/总力矩。 */
		for (int k = 0; k < HAND_BONES; ++k)
			if (s->weights[i][k] > 0 &&
				YMPME_Rigid_AddForceAt(&s->hand.bodies[k],
									   YMPME_VecScale(s->forces[i], s->weights[i][k]),
									   s->patches[i].position) != PME_OK)
				return 0;
	}
	for (int j = 0; j < HAND_JOINTS; ++j)
	{
		float speed =
			YMPME_Clamp((Demo_TargetAngle(&s->hand, j) - s->hand.joints[j].coordinate) * 12, -3, 3);
		if (YMPME_Joint_SetMotor(&s->hand.joints[j], (pme_joint_motor){PME_JOINT_MOTOR_SPEED, speed,
																	   0, effort}) != PME_OK)
			return 0;
	}
	if (YMPME_Joint_Step(s->hand.bodies, HAND_BONES, s->hand.joints, HAND_JOINTS, gravity, dt, 48,
						 s->time) != PME_OK)
		return 0;
	s->hand.time = s->time;
	return YMSKE_PME_Sync(&s->hand.physics) == SKE_OK &&
		   YMSKE_GRE_Update(&s->hand.binding, &s->hand.pose) == SKE_OK;
}
int HandSkin_Step(hand_skin_rig *s) { return step(s, NULL, V(0, 0, 0), V(0, 0, 0), .01f); }
int HandSkin_StepSphere(hand_skin_rig *s, const pme_skin_probe *probe, pme_vec3 omega,
						pme_vec3 gravity, float effort)
{
	return probe && step(s, probe, omega, gravity, effort);
}
void HandSkin_RenderSurface(hand_skin_rig *s)
{
	if (s->patchColors)
		HandUV_RenderColors(s->displayUv ? s->displayUv : &s->uv,
							YMGRE_Camera_GetRenderTarget(s->uvCamera), s->patchColors,
							s->uvSelected);
	else
		HandUV_Render(s->displayUv ? s->displayUv : &s->uv,
					  YMGRE_Camera_GetRenderTarget(s->uvCamera), s->taxels, s->uvForce,
					  s->uvSelected);
	GRE_RenderTarget legend = YMGRE_Camera_GetRenderTarget(s->uvLegend);
	for (int y = 0; y < 16; ++y)
		for (int x = 0; x < HAND_UV_SIZE; ++x)
			legend->data[y * HAND_UV_SIZE + x] =
				GRE_FramePixel_From_RGB24(HandUV_Color((float)x, HAND_UV_SIZE - 1));
	for (int i = 0; i < HAND_POINTS; ++i)
	{
		float pressure = s->vertexArea[i] > 0 ? s->vertexPressure[i] / s->vertexArea[i] : 0;
		float f = s->vertexArea[i] > 0 ? s->vertexForce[i] / s->vertexArea[i] : 0;
		float t = YMPME_Clamp(s->vectorHeat ? f / .1f : pressure / 1000, 0, 1);
		s->hand.object->pointList_wN[i].color =
			s->heat ? s->vertexColors ? s->vertexColors[i]
									  : (GRErgb24){(uint8)(35 + 220 * t),
												   (uint8)(100 + 110 * (1 - fabsf(2 * t - 1))),
												   (uint8)(170 * (1 - t) + 30)}
					: (GRErgb24){255, 255, 255};
	}
	gre_list materials = s->hand.materials;
	if (s->heat)
	{
		s->heatNode.next = materials.listhead;
		s->hand.materials = (gre_list){&s->heatNode, materials.len + 1};
	}
	Demo_Render(&s->hand, 0, s->bones);
	s->hand.materials = materials;
}
void HandSkin_Render(hand_skin_rig *s)
{
	HandSkin_RenderSurface(s);
	gre_line3d lines[96];
	for (int axis = 0; axis < 3; ++axis)
		for (int i = 0; i < 32; ++i)
		{
			float a = i * 6.283185307f / 32, b = (i + 1) * 6.283185307f / 32;
			float ca = cosf(a) * s->probe.radius * 10, sa = sinf(a) * s->probe.radius * 10,
				  cb = cosf(b) * s->probe.radius * 10, sb = sinf(b) * s->probe.radius * 10;
			gre_fvector4d p = {s->probe.center.x * 10, s->probe.center.y * 10,
							   s->probe.center.z * 10, 1},
						  q = p;
			if (axis == 0)
			{
				p.x += ca;
				p.y += sa;
				q.x += cb;
				q.y += sb;
			}
			else if (axis == 1)
			{
				p.x += ca;
				p.z += sa;
				q.x += cb;
				q.z += sb;
			}
			else
			{
				p.y += ca;
				p.z += sa;
				q.y += cb;
				q.z += sb;
			}
			lines[axis * 32 + i] = (gre_line3d){p, q, {245, 245, 250}, 1};
		}
	YMGRE_Camera_LineList_Rendering(s->hand.camera, lines, 96, 0);
}
void HandSkin_Destroy(hand_skin_rig *s)
{
	if (s->uvCamera)
		YMGRE_Free_Camera(s->uvCamera);
	if (s->uvLegend)
		YMGRE_Free_Camera(s->uvLegend);
	s->uvCamera = s->uvLegend = NULL;
	Demo_Destroy(&s->hand);
}
