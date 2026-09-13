#include "ball.h"
#include "YMGRE_Creat.h"
#include "YMGRE_Free.h"
#include <string.h>
#include <stdio.h>
#include "scene_uv_atlas.h"
#define V YMPME_Vec3
static pme_vec3 unit(pme_vec3 p) { return YMPME_VecScale(p, 1 / sqrtf(YMPME_VecDot(p, p))); }
static int triangle(hand_ball_scene *s, pme_vec3 a, pme_vec3 b, pme_vec3 c, int depth, int *at)
{
	if (depth)
	{
		pme_vec3 ab = unit(YMPME_VecAdd(a, b)), bc = unit(YMPME_VecAdd(b, c)),
				 ca = unit(YMPME_VecAdd(c, a));
		return triangle(s, a, ab, ca, depth - 1, at) && triangle(s, ab, b, bc, depth - 1, at) &&
			   triangle(s, ca, bc, c, depth - 1, at) && triangle(s, ab, bc, ca, depth - 1, at);
	}
	int i = (*at)++;
	pme_vec3 n = unit(YMPME_VecCross(YMPME_VecSub(b, a), YMPME_VecSub(c, a)));
	if (YMPME_VecDot(n, a) < 0)
	{
		pme_vec3 tmp = b;
		b = c;
		c = tmp;
		n = YMPME_VecScale(n, -1);
	}
	s->local[i * 3] = a;
	s->local[i * 3 + 1] = b;
	s->local[i * 3 + 2] = c;
	s->normals[i] = n;
	GRE_Polygon4d f = &s->object->polygonList[i];
	f->num = 3;
	f->index = GRE_PolyIndex_Malloc(3 * sizeof(*f->index));
	if (!f->index)
		return 0;
	for (int j = 0; j < 3; j++)
		f->index[j] = (uint16)(i * 3 + j);
	f->planeColor = (GRErgb24){255, 255, 255};
	return 1;
}
static int unwrap(hand_ball_scene *s)
{
	float positions[HAND_POINTS * 3], extent = 0;
	unsigned indices[HAND_FACES * 3];
	int charts[HAND_FACES * 3];
	char error[256] = {0};
	for (int v = 0; v < HAND_POINTS; v++)
	{
		gre_fvector4d p = s->skin.hand.object->pointList[v].pos;
		positions[v * 3] = p.x;
		positions[v * 3 + 1] = p.y;
		positions[v * 3 + 2] = p.z;
		extent = fmaxf(extent, fmaxf(fabsf(p.x), fmaxf(fabsf(p.y), fabsf(p.z))));
	}
	if (extent <= 1e-12f)
		return 0;
	/* 与GRE编辑器相同的归一化及量化，焊合资产中原UV接缝的同位置点。 */
	for (int i = 0; i < HAND_POINTS * 3; i++)
		positions[i] = roundf(positions[i] / extent * 100000) / 100000;
	for (size_t i = 0; i < s->skin.count; i++)
		for (int j = 0; j < 3; j++)
			indices[i * 3 + j] = handTriangles[s->skin.faces[i]][j];
	SceneUvAtlasMesh input = {positions,	  indices, HAND_POINTS, (unsigned)s->skin.count * 3,
							  s->autoCorners, charts};
	if (!SceneUv_AtlasUnwrap(&input, 1, 1, &s->autoCharts, error, sizeof error))
	{
		fprintf(stderr, "GRE hand UV: %s\n", error);
		return 0;
	}
	if (!HandUV_InitCorners(&s->autoUv, s->skin.faces, s->skin.count, s->autoCorners))
		return 0;
	printf("GRE hand UV: %d islands, %zu overlaps, %zu subpixel patches\n", s->autoCharts,
		   s->autoUv.overlapPixels, s->autoUv.uncoveredPatches);
	return 1;
}
const hand_uv_map *HandBall_UV(const hand_ball_scene *s)
{
	return s->sourceUv ? &s->skin.uv : &s->autoUv;
}
int HandBall_Reset(hand_ball_scene *s)
{
	if (!HandSkin_Select(&s->skin, 0))
		return 0;
	s->skin.time = 0;
	s->skin.hand.time = 0;
	s->skin.peakPressure = 0;
	s->skin.normalLoad = 0;
	memset(&s->skin.loadReading, 0, sizeof s->skin.loadReading);
	s->skin.touch = (pme_surface_tactile_frame){0};
	s->skin.touch.cells = s->skin.taxels;
	s->skin.touch.capacity = HAND_FACES;
	memset(s->skin.taxels, 0, sizeof s->skin.taxels);
	memset(s->skin.forces, 0, sizeof s->skin.forces);
	memset(&s->skin.reaction, 0, sizeof s->skin.reaction);
	memset(s->skin.vertexPressure, 0, sizeof s->skin.vertexPressure);
	memset(s->skin.vertexArea, 0, sizeof s->skin.vertexArea);
	memset(s->skin.vertexForce, 0, sizeof s->skin.vertexForce);
	pme_force_device_desc desc = s->skin.loadCell.desc;
	if (YMPME_ForceDevice_Init(&s->skin.loadCell, &desc, 734) != PME_OK)
		return 0;
	s->radius = .032f;
	s->floorZ = -.06f;
	s->wristTarget = V(0, 0, 0);
	s->tilt = 0;
	s->grip = .85f;
	s->motion = 0;
	s->motionTime = 0;
	s->gravity = 1;
	s->lost = 0;
	s->floorForce = 0;
	s->totalNormal = 0;
	/* 有质量的手腕通过PD伺服跟随目标；球没有位置锁定或抓取约束。 */
	if (YMPME_Rigid_Init(&s->skin.hand.bodies[0], .3f, V(.002f, .002f, .002f), V(0, 0, 0),
						 YMPME_Quaternion_Identity()) != PME_OK)
		return 0;
	float mass = .035f, inertia = .4f * mass * s->radius * s->radius;
	if (YMPME_Rigid_Init(&s->ball, mass, V(inertia, inertia, inertia), V(-.006f, .085f, .053f),
						 YMPME_Quaternion_Identity()) != PME_OK)
		return 0;
	s->skin.probe = (pme_skin_probe){s->ball.position, s->ball.velocity, s->radius};
	s->skin.contact = (pme_skin_contact_desc){400000, 2200, .8f, 1800, 10000};
	return 1;
}
int HandBall_Init(hand_ball_scene *s)
{
	memset(s, 0, sizeof *s);
	if (!HandSkin_Init(&s->skin) || !unwrap(s))
		return 0;
	s->object = YMGRE_Creat_Object(BALL_POINTS, BALL_FACES, "play_ball", "play_ball_material");
	s->material = YMGRE_Creat_Material("play_ball_material");
	if (!s->object || !s->material || !YMGRE_Object_EnableVertexAttributes(s->object))
		return 0;
	s->material->diffuse = (GRErgb24){255, 255, 255};
	s->material->unlit = 1;
	/* 原生材质默认无纹理；1x1白纹理显式传递顶点色。 */
	s->material->pixel = GRE_malloc1(sizeof(GRErgb24));
	if (!s->material->pixel)
		return 0;
	s->material->pixel[0] = (GRErgb24){255, 255, 255};
	s->material->width = s->material->height = 1;
	int at = 0;
	pme_vec3 poles[2] = {V(0, 0, 1), V(0, 0, -1)},
			 ring[4] = {V(1, 0, 0), V(0, 1, 0), V(-1, 0, 0), V(0, -1, 0)};
	for (int p = 0; p < 2; p++)
		for (int k = 0; k < 4; k++)
			if (!triangle(s, poles[p], ring[k], ring[(k + 1) % 4], 2, &at))
				return 0;
	s->object->renderMode = GRE_RenderMode_Vertex;
	s->object->boundType = GRE_Bounding_Sphere_R;
	s->objectNode = (gre_listnode){sizeof(*s->object), s->object, NULL};
	s->skin.hand.objectNode.next = &s->objectNode;
	s->skin.hand.objects.len = 2;
	s->materialNode =
		(gre_listnode){sizeof(*s->material), s->material, s->skin.hand.materials.listhead};
	s->skin.hand.materials.listhead = &s->materialNode;
	s->skin.hand.materials.len++;
	s->arrows = 0;
	s->skin.vectorHeat = 1;
	s->skin.uvForce = 2;
	s->skin.hand.orbit = .6f;
	s->skin.hand.elevation = .25f;
	s->skin.hand.distance = 3.2f;
	return HandBall_Reset(s);
}
void HandBall_Nudge(hand_ball_scene *s, pme_vec3 impulse)
{
	if (YMPME_VecFinite(impulse))
		YMPME_Rigid_Impulse(&s->ball, impulse, V(0, 0, 0));
}
int HandBall_Step(hand_ball_scene *s)
{
	const float dt = 1.f / 240;
	pme_vec3 gravity = s->gravity ? V(0, 0, -9.81f) : V(0, 0, 0);
	pme_rigid_body *wrist = &s->skin.hand.bodies[0];
	pme_vec3 target = s->wristTarget;
	float tilt = s->tilt;
	if (s->motion)
	{
		s->motionTime += dt;
		target.x += .014f * sinf((float)s->motionTime * 1.4f);
		target.z += .009f * (1 - cosf((float)s->motionTime * 1.1f));
		tilt += .18f * sinf((float)s->motionTime);
	}
	pme_vec3 force = YMPME_VecSub(YMPME_VecScale(YMPME_VecSub(target, wrist->position), 800),
								  YMPME_VecScale(wrist->velocity, 32));
	force = YMPME_VecSub(force, YMPME_VecScale(gravity, .3f + .008f * 15));
	pme_quaternion q;
	if (YMPME_Quaternion_AxisAngle(V(0, 1, 0), tilt, &q) != PME_OK)
		return 0;
	pme_vec3 error = YMPME_Quaternion_Log(
		YMPME_Quaternion_Multiply(q, YMPME_Quaternion_Conjugate(wrist->orientation)));
	pme_vec3 torque =
		YMPME_VecSub(YMPME_VecScale(error, 2), YMPME_VecScale(wrist->angularVelocity, .12f));
	if (YMPME_Rigid_AddForceAt(wrist, force, wrist->position) != PME_OK ||
		YMPME_Rigid_AddTorque(wrist, torque) != PME_OK)
		return 0;
	for (int i = 0; i < 5; i++)
		Demo_SetFingerCurl(&s->skin.hand, i, s->grip);
	pme_skin_probe probe = {s->ball.position, s->ball.velocity, s->radius};
	if (!HandSkin_StepSphere(&s->skin, &probe, s->ball.angularVelocity, gravity, .035f))
		return 0;
	s->totalNormal = 0;
	for (size_t i = 0; i < s->skin.count; i++)
		s->totalNormal += s->skin.taxels[i].normalForce;
	if (YMPME_Rigid_AddForceAt(&s->ball, s->skin.reaction.probeWrench.force, s->ball.position) !=
			PME_OK ||
		YMPME_Rigid_AddTorque(&s->ball, s->skin.reaction.probeWrench.torque) != PME_OK)
		return 0;
	/* 静止桌面：柔性法向接触+库仑上限黏性摩擦，力矩来自球底接触点。 */
	s->floorForce = 0;
	float depth = s->floorZ + s->radius - s->ball.position.z;
	if (depth > 0)
	{
		s->floorForce = fmaxf(0, 600 * depth - 5 * s->ball.velocity.z);
		pme_vec3 r = V(0, 0, -s->radius),
				 v = YMPME_VecAdd(s->ball.velocity, YMPME_VecCross(s->ball.angularVelocity, r));
		v.z = 0;
		float speed = sqrtf(YMPME_VecDot(v, v));
		pme_vec3 f = V(0, 0, s->floorForce);
		if (speed > 1e-8f)
			f = YMPME_VecSub(f, YMPME_VecScale(v, fminf(.65f * s->floorForce, 2 * speed) / speed));
		if (YMPME_Rigid_AddForceAt(&s->ball, f, YMPME_VecAdd(s->ball.position, r)) != PME_OK)
			return 0;
	}
	if (YMPME_Rigid_Step(&s->ball, gravity, dt, s->skin.time) != PME_OK)
		return 0;
	s->lost = YMPME_VecDot(YMPME_VecSub(s->ball.position, wrist->position),
						   YMPME_VecSub(s->ball.position, wrist->position)) > .09f;
	return Demo_Validate(&s->skin.hand) && YMPME_Rigid_IsValid(&s->ball);
}
void HandBall_Render(hand_ball_scene *s)
{
	pme_rotation rotation = YMPME_Quaternion_Rotation(s->ball.orientation);
	for (int i = 0; i < BALL_FACES; i++)
	{
		pme_vec3 n = YMPME_Rotation_Apply(rotation, s->normals[i]);
		s->object->polygonList[i].pN = (gre_fvector4d){n.x, n.y, n.z, 0};
		float shade = .48f + .52f * fmaxf(0, YMPME_VecDot(n, unit(V(-.4f, .6f, 1))));
		GRErgb24 color =
			i / 16 % 2
				? (GRErgb24){(uint8)(245 * shade), (uint8)(168 * shade), (uint8)(55 * shade)}
				: (GRErgb24){(uint8)(230 * shade), (uint8)(220 * shade), (uint8)(185 * shade)};
		for (int j = 0; j < 3; j++)
		{
			int v = i * 3 + j;
			pme_vec3 p = YMPME_VecScale(
				YMPME_VecAdd(
					s->ball.position,
					YMPME_VecScale(YMPME_Rotation_Apply(rotation, s->local[v]), s->radius)),
				10);
			s->object->pointList[v].pos = (gre_fvector4d){p.x, p.y, p.z, 1};
			s->object->pointList_wN[v].base = s->object->pointList[v];
			s->object->pointList_wN[v].normal = (gre_fvector4d){n.x, n.y, n.z, 0};
			s->object->pointList_wN[v].color = color;
		}
	}
	s->object->WorldCoordinate = (gre_fvector4d){s->ball.position.x * 10, s->ball.position.y * 10,
												 s->ball.position.z * 10, 1};
	s->object->BoundingSphereR = s->radius * 10;
	s->object->isVisible = !s->wire;
	s->skin.displayUv = HandBall_UV(s);
	if (s->colorMode == 0)
		HandBall_UpdateRGB(s);
	s->skin.patchColors = s->colorMode == 0 ? s->rgbPatch : NULL;
	s->skin.vertexColors = s->colorMode == 0 ? s->rgbVertex : NULL;
	HandSkin_RenderSurface(&s->skin);
	if (s->colorMode == 0)
		HandBall_RGBLegend(s);
	gre_line3d lines[BALL_FACES * 3 + 42];
	int count = 0;
	if (s->wire)
		for (int i = 0; i < BALL_FACES; i++)
			for (int j = 0; j < 3; j++)
				lines[count++] = (gre_line3d){s->object->pointList[i * 3 + j].pos,
											  s->object->pointList[i * 3 + (j + 1) % 3].pos,
											  {255, 205, 95},
											  1};
	/* 桌面网格提供深度和落球参照；与球使用相同世界Z。 */
	for (int i = -10; i <= 10; i++)
	{
		float k = i * .2f, z = s->floorZ * 10;
		lines[count++] = (gre_line3d){{-2, k + 1, z, 1}, {2, k + 1, z, 1}, {48, 65, 80}, 1};
		lines[count++] = (gre_line3d){{k, -1, z, 1}, {k, 3, z, 1}, {48, 65, 80}, 1};
	}
	YMGRE_Camera_LineList_Rendering(s->skin.hand.camera, lines, (uint32)count, 1);
	HandBall_RenderForces(s);
}
void HandBall_Destroy(hand_ball_scene *s)
{
	if (s->object)
		YMGRE_Free_Object(s->object);
	if (s->material)
		YMGRE_Free_Material(s->material);
	s->object = NULL;
	s->material = NULL;
	HandSkin_Destroy(&s->skin);
}
