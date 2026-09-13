#include "scene.h"
#include "YMGRE_BasicMesh_Gener.h"
#include "YMGRE_Rendering_Pipeline.h"
#include "YMGRE_Camera.h"
#include "YMGRE_Creat.h"
#include "YMGRE_Free.h"
#include <stdlib.h>
#include <string.h>
#define U 60.f
#define V(x, y, z) YMPME_Vec3(x, y, z)
static int add(joint_scene *s, GRE_Object4d o, int owner, pme_vec3 offset)
{
	joint_part *p;
	int i;
	if (!o)
		return 0;
	if (s->count >= 18)
	{
		YMGRE_Free_Object(o);
		return 0;
	}
	p = &s->parts[s->count++];
	p->object = o;
	p->owner = owner;
	p->offset = offset;
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
static int box(joint_scene *s, pme_vec3 size, GRErgb24 color, int owner, pme_vec3 offset)
{
	return add(s, YMGRE_MeshGener_Box(size.x * U, size.y * U, size.z * U, color, "joint-part", ""),
			   owner, offset);
}
int JointScene_Init(joint_scene *s, int kind)
{
	const GRErgb24 dark = {48, 65, 82}, steel = {151, 177, 191}, blue = {51, 171, 213},
				   gold = {237, 173, 57}, red = {232, 86, 83};
	memset(s, 0, sizeof(*s));
	s->kind = kind;
	if (!add(s, YMGRE_MeshGener_RectPlane(5 * U, 4 * U, 10, 8, (GRErgb24){55, 67, 76}, "floor", ""),
			 -1, V(0, 0, 0)))
		return 0;
	if (kind == 0)
	{
		if (!box(s, V(.16f, 2.9f, .16f), dark, -1, V(0, 1.45f, .45f)) ||
			!box(s, V(.7f, .12f, .8f), steel, -1, V(0, 2.5f, .2f)) ||
			!add(s, YMGRE_MeshGener_Sphere(.13f * U, 12, 16, red, "pivot", ""), -1,
				 V(0, 2.5f, 0)) ||
			!box(s, V(.07f, 1.5f, .07f), steel, 0, V(0, .75f, 0)) ||
			!add(s, YMGRE_MeshGener_Sphere(.22f * U, 18, 24, gold, "load", ""), 0, V(0, 0, 0)) ||
			!box(s, V(.025f, 1.5f, .025f), blue, 2, V(0, .75f, .08f)))
			return 0;
	}
	else if (kind == 1)
	{
		if (!box(s, V(.36f, 1.8f, .5f), dark, -1, V(-.65f, .9f, 0)) ||
			!box(s, V(.32f, .38f, .5f), red, -1, V(-.45f, 1.8f, 0)) ||
			!box(s, V(1.8f, .12f, .25f), gold, 0, V(0, 0, 0)) ||
			!box(s, V(.05f, .7f, .05f), red, 3, V(1.35f, 2.25f, 0)) ||
			!add(s, YMGRE_MeshGener_Sphere(.10f * U, 10, 12, red, "load-point", ""), 3,
				 V(1.35f, 1.9f, 0)))
			return 0;
	}
	else
	{
		if (!box(s, V(3.2f, .1f, .1f), steel, -1, V(0, 1.2f, -.24f)) ||
			!box(s, V(3.2f, .1f, .1f), steel, -1, V(0, 1.2f, .24f)) ||
			!box(s, V(.18f, 1.2f, .65f), dark, -1, V(-1.55f, .6f, 0)) ||
			!box(s, V(.18f, 1.2f, .65f), dark, -1, V(1.55f, .6f, 0)) ||
			!box(s, V(.12f, .42f, .65f), red, -1, V(-1.55f, 1.4f, 0)) ||
			!box(s, V(.12f, .42f, .65f), red, -1, V(1.55f, 1.4f, 0)) ||
			!box(s, V(.6f, .4f, .6f), blue, 0, V(0, 0, 0)) ||
			!box(s, V(.08f, .015f, .48f), gold, 0, V(0, .21f, 0)))
			return 0;
	}
	s->ambient = YMGRE_Creat_Light(0, GRE_GlobalLight, (GRErgb24){255, 255, 255}, .65f);
	s->key = YMGRE_Creat_Light(1, GRE_PointLight, (GRErgb24){255, 244, 225}, .7f);
	if (!s->ambient || !s->key)
		return 0;
	s->key->pos = (gre_fvector4d){-180, 350, -220, 1};
	s->key->proper.kc0 = 1;
	s->key->proper.kc1 = s->key->proper.kc2 = 0;
	YMGRE_List_Append(&s->lights, sizeof(*s->ambient), s->ambient);
	YMGRE_List_Append(&s->lights, sizeof(*s->key), s->key);
	s->camera = YMGRE_Creat_Camera(0, 400, 400, 26, 26, 26, 26);
	s->workspace = YMGRE_Creat_RenderWorkspace();
	if (!s->camera || !s->workspace)
		return 0;
	YMGRE_Camera_Frustum_Init(s->camera, 1, 2000);
	return 1;
}
void JointScene_Render(joint_scene *s, const joint_lab_state *state, int view, int wire)
{
	const pme_rigid_body *body = s->kind == 0	? &state->pendulum.body
								 : s->kind == 1 ? &state->fixedBody
												: &state->sliderBody;
	int i, k;
	gre_fvector4d eye, target;
	for (i = 0; i < s->count; ++i)
	{
		joint_part *p = &s->parts[i];
		GRE_Object4d o = p->object;
		pme_rotation r = YMPME_Rotation_Identity();
		pme_vec3 position = V(0, 0, 0), center;
		if (p->owner == 0)
		{
			r = YMPME_Quaternion_Rotation(body->orientation);
			position = body->position;
		}
		if (p->owner == 2)
		{
			pme_quaternion q;
			float angle =
				state->automatic ? .95f * sinf((float)state->pendulum.time * .7f) : state->target;
			YMPME_Quaternion_AxisAngle(V(0, 0, 1), angle, &q);
			r = YMPME_Quaternion_Rotation(q);
			position =
				YMPME_VecAdd(state->pendulum.config.anchor,
							 YMPME_Rotation_Apply(r, V(0, -state->pendulum.config.length, 0)));
		}
		center = YMPME_VecAdd(position, YMPME_Rotation_Apply(r, p->offset));
		o->WorldCoordinate = (gre_fvector4d){center.x * U, center.y * U, center.z * U, 1};
		o->wireFrame = (uint8)(wire && p->owner == 0);
		o->isVisible = (int8)(p->owner != 3 || state->extraLoad);
		for (k = 0; k < o->pointNum; ++k)
		{
			pme_vec3 v =
				YMPME_Rotation_Apply(r, V(p->local[k].pos.x, p->local[k].pos.y, p->local[k].pos.z));
			o->pointList[k] = p->local[k];
			o->pointList[k].pos =
				(gre_fvector4d){v.x + center.x * U, v.y + center.y * U, v.z + center.z * U, 1};
		}
		for (k = 0; k < o->polygonNum; ++k)
		{
			pme_vec3 n = YMPME_Rotation_Apply(r, V(p->normal[k].x, p->normal[k].y, p->normal[k].z));
			o->polygonList[k].pN = (gre_fvector4d){n.x, n.y, n.z, 0};
		}
	}
	eye = view ? (gre_fvector4d){0, 1.55f * U, -5.1f * U, 1}
			   : (gre_fvector4d){2.5f * U, 2.65f * U, -5.1f * U, 1};
	target = (gre_fvector4d){0, 1.4f * U, 0, 1};
	YMGRE_UVNCamera_PositionInit(s->camera, &eye, &target, NULL, 0);
	YMGRE_Camera_TanglePipline_RenderingWithWorkspace(s->camera, &s->lights, &s->objects,
													  &s->materials, s->workspace);
}
static void keep(void *p) { (void)p; }
void JointScene_Destroy(joint_scene *s)
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
