#include "rig.h"
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
int HumanSelf_Init(human_skin_rig *s)
{
	human_self_touch *t = &s->self;
	t->desc = YMPME_SkinSelfContact_DefaultDesc();
	t->desc.sourceRegions = 1u << 1;
	t->desc.targetRegions = 1u << 2;
	t->desc.allow[1] = 1u << 2;
	t->desc.allow[2] = 1u << 1;
	t->desc.material = (pme_skin_contact_desc){160000, 200, .55f, 2400, 6000};
	t->cycle = 1;
	s->contactMarks = 1;
	/* This asset's Arm.L -> Hand.L -> Wrist.L is shoulder -> elbow -> wrist. */
	if (strcmp(s->human.boneNames[23].text, "Arm .L") ||
		strcmp(s->human.boneNames[25].text, "Wrist .L"))
		return 0;
	pme_vec3 center = V(0, 0, 0);
	float sum = 0;
	for (unsigned v = 0; v < s->human.asset.info.vertexCount; v++)
	{
		const ske_vertex *p = &s->human.vertices[v];
		float w = 0;
		for (int k = 0; k < YMSKE_MAX_INFLUENCES; k++)
			if (p->weights.bone[k] == 25)
				w += p->weights.weight[k];
		center = add(center, mul(V(p->position.x, p->position.y, p->position.z), w));
		sum += w;
	}
	if (sum <= 0)
		return 0;
	center = mul(center, 1 / sum);
	ske_vec3 local =
		YMSKE_Mat_Point(s->human.bones[25].inverseBind, (ske_vec3){center.x, center.y, center.z});
	t->palmLocal = V(local.x, .38f, local.z);
	for (size_t i = 0; i < s->count; i++)
	{
		float hand = 0, body = 0;
		for (int j = 0; j < 3; j++)
			for (int k = 0; k < YMSKE_MAX_INFLUENCES; k++)
			{
				const ske_weights *w = &s->human.vertices[s->triangle[i][j]].weights;
				int b = w->bone[k];
				if (b >= 25 && b <= 39)
					hand += w->weight[k] / 3;
				if (b >= 17 && b <= 19)
					body += w->weight[k] / 3;
			}
		pme_vec3 at = s->patches[i].position;
		int torso =
			fabsf(at.x) < .115f && at.y > .095f && at.y < .34f && s->patches[i].normal.z > .3f;
		t->faces[i].region = hand > .65f ? 1 : (body > .15f && torso) ? 2 : 0;
	}
	return 1;
}
int HumanSelf_Enable(human_skin_rig *s, int enabled)
{
	if (!HumanSkin_Region(s, 0))
		return 0;
	human_self_touch *t = &s->self;
	s->contactView.overview = 3;
	t->enabled = !!enabled;
	t->clock = 0;
	t->cycle = 1;
	t->filtered = 0;
	t->correction = 0;
	t->anchor = V(.020f, .215f, .070f);
	t->desired = t->corrected = add(t->anchor, V(0, 0, .13f));
	s->motion = 0;
	s->rub = 1;
	s->angle = 0;
	s->loadBone = 25;
	s->selectedUv = -1;
	s->human.orbit = .32f;
	s->human.elevation = .08f;
	s->human.distance = 12;
	s->gain = enabled ? .02f : 2;
	memset(t->moments, 0, sizeof t->moments);
	return 1;
}
pme_vec3 HumanSelf_Target(human_skin_rig *s)
{
	human_self_touch *t = &s->self;
	float q = fmodf(t->clock, 10), gap;
	if (!t->cycle)
	{
		t->phase = 4;
		gap = -s->indentation;
	}
	else if (q < 2)
	{
		t->phase = 0;
		float a = q / 2;
		a = a * a * (3 - 2 * a);
		gap = .13f * (1 - a) - .012f * a;
	}
	else if (q < 6)
	{
		t->phase = 1;
		gap = -.012f;
	}
	else if (q < 8)
	{
		t->phase = 2;
		float a = (q - 6) / 2;
		a = a * a * (3 - 2 * a);
		gap = -.012f * (1 - a) + .13f * a;
	}
	else
	{
		t->phase = 3;
		gap = .13f;
	}
	pme_vec3 p = t->anchor;
	if (s->rub)
	{
		p.x += .016f * sinf(t->clock * 1.2f);
		p.y += .012f * sinf(t->clock * .8f);
	}
	p.x += s->offsetU;
	p.y += s->offsetV;
	p.z += gap;
	t->desired = p;
	return p;
}
int HumanSelf_Pose(human_skin_rig *s, pme_vec3 palm)
{
	demo_scene *h = &s->human;
	human_self_touch *t = &s->self;
	for (unsigned i = 0; i < h->asset.info.boneCount; i++)
		h->poseBones[i].local = h->animation.samples[i];
	if (YMSKE_Pose_Update(&h->pose) != SKE_OK)
		return 0;
	/* Palm plane parallel to the chest, fingers diagonally down toward the midline. */
	ske_mat34 wrist = {{{-.6f, -.8f, 0, 0}, {.8f, -.6f, 0, 0}, {0, 0, 1, 0}}};
	ske_vec3 off =
		YMSKE_Mat_Point(wrist, (ske_vec3){t->palmLocal.x, t->palmLocal.y, t->palmLocal.z});
	pme_vec3 target = sub(palm, V(off.x * .1f, off.y * .1f, off.z * .1f));
	pme_vec3 shoulder = position(h, 23);
	float a = length(sub(position(h, 24), shoulder)),
		  b = length(sub(position(h, 25), position(h, 24)));
	pme_vec3 ray = sub(target, shoulder);
	float dist = YMPME_Clamp(length(ray), fabsf(a - b) + .001f, a + b - .001f);
	pme_vec3 along = unit(ray), pole = V(.8f, -1, .5f);
	pole = unit(sub(pole, mul(along, dot(pole, along))));
	float x = (a * a - b * b + dist * dist) / (2 * dist), y = sqrtf(fmaxf(0, a * a - x * x));
	pme_vec3 elbow = add(shoulder, add(mul(along, x), mul(pole, y)));
	if (!aim(h, 23, 24, elbow) || !aim(h, 24, 25, add(shoulder, mul(along, dist))) ||
		!worldRotation(h, 25, wrist))
		return 0;
	t->ikError = length(sub(position(h, 25), target));
	t->corrected = palm;
	return YMSKE_GRE_Update(&h->binding, &h->pose) == SKE_OK;
}
int HumanSelf_Contact(human_skin_rig *s)
{
	human_self_touch *t = &s->self;
	for (size_t i = 0; i < s->count; i++)
	{
		t->faces[i].surface = s->patches[i];
		for (int j = 0; j < 3; j++)
		{
			gre_fvector4d v = s->human.object->pointList[s->triangle[i][j]].pos;
			t->faces[i].vertex[j] = V(v.x * .1f, v.y * .1f, v.z * .1f);
		}
	}
	t->desc.allow[1] = t->filtered ? 0 : 1u << 2;
	return YMPME_SkinSelfContact_Solve(&t->desc, t->faces, s->count, s->probe.center, s->forces,
									   t->moments, t->targets, DEMO_FACES, &t->result) == PME_OK;
}
const char *HumanSelf_Phase(const human_skin_rig *s)
{
	static const char *names[] = {"APPROACH", "TOUCH / SLIDE", "LIFT", "RELEASED", "MANUAL"};
	return names[s->self.phase];
}
