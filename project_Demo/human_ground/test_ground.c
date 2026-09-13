#include "rig.h"
#include "observation.h"
#include <stdio.h>
#include <string.h>
#define CHECK(x)                                                                                   \
	do                                                                                             \
	{                                                                                              \
		if (!(x))                                                                                  \
		{                                                                                          \
			fprintf(stderr, "ground FAIL %d: %s (time %.3f L/R %.3f/%.3f clearance %.5f/%.5f)\n",  \
					__LINE__, #x, s.time, s.ground.footLoad[0], s.ground.footLoad[1],              \
					s.ground.footClearance[0], s.ground.footClearance[1]);                         \
			goto fail;                                                                             \
		}                                                                                          \
	} while (0)
static human_skin_rig s;
static uint8_t rgb[512 * 512 * 3], saved[512 * 512 * 3];
static pme_vec3 local[DEMO_FACES];
static uint16_t ids[512 * 512];
static uint8_t gridRgb[64 * 64 * 3], gridSaved[64 * 64 * 3];
static uint16_t gridIds[64 * 64];
static float initialAreas[64 * 64], savedAreas[64 * 64];
static float norm(pme_vec3 v) { return sqrtf(YMPME_VecDot(v, v)); }
static uint64_t imageHash(void)
{
	GRE_RenderTarget t = YMGRE_Camera_GetRenderTarget(s.human.camera);
	const unsigned char *p = (const unsigned char *)t->data;
	uint64_t h = 14695981039346656037ull;
	for (size_t i = 0; i < DEMO_WIDTH * DEMO_HEIGHT * sizeof(*t->data); i++)
		h = (h ^ p[i]) * 1099511628211ull;
	return h;
}
static int checkAreas(int initial)
{
	const float *a = HumanSkin_GridAreas(&s);
	if (!a)
		return 0;
	for (size_t c = 0; c < 64 * 64; c++)
	{
		uint16_t id = s.contactView.gridIds[c];
		if (id == 65535)
		{
			if (a[c] != 0)
				return 0;
			continue;
		}
		if (!(a[c] > 0))
			return 0;
		if (!initial)
		{
			if (a[c] != initialAreas[c])
				return 0;
			continue;
		}
		if (a[c] != s.patches[id].area)
			return 0;
		gre_fvector4d p[3];
		for (int j = 0; j < 3; j++)
			p[j] = s.human.object->pointList[s.triangle[id][j]].pos;
		pme_vec3 e =
			YMPME_Vec3((p[1].x - p[0].x) * .1f, (p[1].y - p[0].y) * .1f, (p[1].z - p[0].z) * .1f);
		pme_vec3 f =
			YMPME_Vec3((p[2].x - p[0].x) * .1f, (p[2].y - p[0].y) * .1f, (p[2].z - p[0].z) * .1f);
		if (fabsf(a[c] - .5f * norm(YMPME_VecCross(e, f))) > a[c] * .0002f)
			return 0;
	}
	return 1;
}
int main(void)
{
	CHECK(HumanSkin_GridAreas(NULL) == NULL);
	CHECK(HumanSkin_Init(&s));
	CHECK(checkAreas(1));
	const float *areaPointer = HumanSkin_GridAreas(&s);
	memcpy(initialAreas, areaPointer, sizeof initialAreas);
	CHECK(HumanGround_Enable(&s, 0));
	memcpy(ids, HumanSkin_ObservationMap(&s), sizeof ids);
	memcpy(gridIds, HumanSkin_GridObservationMap(&s), sizeof gridIds);
	unsigned unique[DEMO_FACES] = {0};
	size_t populated = 0;
	for (size_t c = 0; c < 64 * 64; c++)
	{
		uint16_t f = gridIds[c];
		if (f == 65535)
			continue;
		CHECK(f < s.count && ++unique[f] == 1);
		CHECK(s.contactView.faceCells[f] == c);
		populated++;
		CHECK(HumanSkin_PickUV(&s, (c % 64) * 8 + 4, (c / 64) * 8 + 4) == f);
	}
	CHECK(populated == s.count);
	for (size_t f = 0; f < s.count; f++)
		CHECK(unique[f] == 1);

	int stance[2] = {0}, flight[2] = {0};
	float minimum = 1, maximum[2] = {0};
	for (int i = 0; i < 960; i++)
	{
		CHECK(HumanSkin_Step(&s));
		for (int side = 0; side < 2; side++)
		{
			minimum = fminf(minimum, s.ground.footClearance[side]);
			CHECK(s.ground.footClearance[side] > -.001f);
			if (s.ground.lift[side] > .03f)
			{
				CHECK(s.ground.footLoad[side] == 0);
				flight[side]++;
			}
			if (s.ground.lift[side] < .0001f && s.ground.footLoad[side] > 0)
				stance[side]++;
			maximum[side] = fmaxf(maximum[side], s.ground.footLoad[side]);
		}
		CHECK(s.ground.otherLoad == 0);
		CHECK(norm(YMPME_VecAdd(s.touch.wrench.force, s.reaction.probeWrench.force)) < 1e-4f);
		CHECK(norm(YMPME_VecAdd(s.touch.wrench.torque, s.reaction.probeWrench.torque)) < 1e-4f);
	}
	CHECK(checkAreas(0));
	CHECK(areaPointer == HumanSkin_GridAreas(&s));
	CHECK(!memcmp(initialAreas, areaPointer, sizeof initialAreas));
	int deformed = 0;
	for (size_t p = 0; p < s.count; p++)
		deformed += fabsf(s.patches[p].area - areaPointer[s.contactView.faceCells[p]]) > 1e-8f;
	CHECK(deformed > 0);
	CHECK(stance[0] > 100 && stance[1] > 100 && flight[0] > 100 && flight[1] > 100);
	CHECK(HumanGround_Enable(&s, 1));
	CHECK(HumanSkin_Step(&s));
	CHECK(s.ground.footLoad[0] > 0 && s.ground.footLoad[1] > 0);
	pme_sensor_stamp stamp;
	CHECK(HumanSkin_Observe(&s, rgb, sizeof rgb, local, DEMO_FACES, &stamp));
	CHECK(stamp.valid);
	uint8_t seen[DEMO_FACES] = {0};
	int activePixels[2] = {0};
	for (size_t i = 0; i < 512 * 512; i++)
	{
		int p = ids[i];
		if (p == 65535)
		{
			CHECK(rgb[i * 3] == 0 && rgb[i * 3 + 1] == 0 && rgb[i * 3 + 2] == 0);
			continue;
		}
		CHECK(p >= 0 && (size_t)p < s.count);
		seen[p] = 1;
		if (s.ground.foot[p] && rgb[i * 3] > 0)
			activePixels[s.ground.foot[p] - 1]++;
	}
	for (size_t p = 0; p < s.count; p++)
		CHECK(seen[p]);
	CHECK(activePixels[0] > 0 && activePixels[1] > 0);
	memcpy(saved, rgb, sizeof rgb);
	CHECK(HumanSkin_ObserveGrid(&s, gridRgb, sizeof gridRgb, local, DEMO_FACES, &stamp));
	memcpy(gridSaved, gridRgb, sizeof gridRgb);
	int gridActive[2] = {0};
	for (size_t c = 0; c < 64 * 64; c++)
		if (gridIds[c] != 65535 && s.ground.foot[gridIds[c]] && gridRgb[c * 3] > 0)
			gridActive[s.ground.foot[gridIds[c]] - 1]++;
	CHECK(gridActive[0] > 0 && gridActive[1] > 0);

	memcpy(savedAreas, areaPointer, sizeof savedAreas);
	s.uvZoom = 4;
	s.selectedUv = 123;
	s.gain = .1f;
	s.colorMode = 2;
	s.contactView.showGroups = 1;
	s.contactView.overview = 2;
	Demo_ViewMove(&s.human, .2f, .1f, 0);
	HumanSkin_Render(&s);
	uint64_t plain = imageHash();
	s.pickTest = 1;
	HumanSkin_Render(&s);
	CHECK(imageHash() != plain);
	s.pickTest = 0;
	HumanSkin_Render(&s);
	CHECK(imageHash() == plain);
	s.pickTest = 1;
	s.selectedUv = -1;
	HumanSkin_Render(&s);
	CHECK(imageHash() == plain);
	s.selectedUv = 123;
	HumanSkin_Render(&s);
	CHECK(HumanSkin_Observe(&s, rgb, sizeof rgb, local, DEMO_FACES, &stamp));
	CHECK(!memcmp(saved, rgb, sizeof rgb));
	CHECK(!memcmp(ids, HumanSkin_ObservationMap(&s), sizeof ids));
	CHECK(HumanSkin_ObserveGrid(&s, gridRgb, sizeof gridRgb, local, DEMO_FACES, &stamp));
	CHECK(!memcmp(gridSaved, gridRgb, sizeof gridRgb));
	CHECK(!memcmp(gridIds, HumanSkin_GridObservationMap(&s), sizeof gridIds));
	CHECK(!memcmp(savedAreas, areaPointer, sizeof savedAreas));
	CHECK(checkAreas(0));

	CHECK(HumanGround_Enable(&s, 2));
	for (int i = 0; i < 360; i++)
	{
		CHECK(HumanSkin_Step(&s));
		CHECK(s.ground.footClearance[0] > -.001f && s.ground.footClearance[1] > -.001f);
		if (i % 60 == 0)
			CHECK(checkAreas(0));
	}
	CHECK(HumanGround_Enable(&s, 3));
	CHECK(HumanSkin_Step(&s));
	CHECK(!s.reaction.contacts && s.totalNormal == 0);
	CHECK(HumanSkin_Observe(&s, rgb, sizeof rgb, local, DEMO_FACES, &stamp));
	for (size_t i = 0; i < 512 * 512; i++)
		if (ids[i] != 65535)
			CHECK(rgb[i * 3] == 0 && rgb[i * 3 + 1] == 128 && rgb[i * 3 + 2] == 128);
	CHECK(HumanSkin_ObserveGrid(&s, gridRgb, sizeof gridRgb, local, DEMO_FACES, &stamp));
	for (size_t c = 0; c < 64 * 64; c++)
		if (gridIds[c] != 65535)
			CHECK(gridRgb[c * 3] == 0 && gridRgb[c * 3 + 1] == 128 && gridRgb[c * 3 + 2] == 128);
	printf("Ground PASS: stance %d/%d, swing %d/%d, max load %.2f/%.2f N, min clearance %.3f mm, "
		   "RGB foot pixels %d/%d; fixed observation covers all %zu faces\n",
		   stance[0], stance[1], flight[0], flight[1], maximum[0], maximum[1], minimum * 1000,
		   activePixels[0], activePixels[1], s.count);
	HumanSkin_Destroy(&s);
	return 0;
fail:
	HumanSkin_Destroy(&s);
	return 1;
}
