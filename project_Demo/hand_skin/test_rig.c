#include "rig.h"
#include <stdio.h>
#include <string.h>
#include <float.h>
#define CHECK(x)                                                                                   \
	do                                                                                             \
	{                                                                                              \
		if (!(x))                                                                                  \
		{                                                                                          \
			fprintf(stderr, "FAIL %d: %s\n", __LINE__, #x);                                        \
			return 1;                                                                              \
		}                                                                                          \
	} while (0)
#define NEAR(a, b) (fabs((double)(a) - (b)) < 1e-5)

#ifdef WRAP_ALLOC
static int tracking;
static size_t calls;
void *__real_malloc(size_t);
void *__real_calloc(size_t, size_t);
void *__real_realloc(void *, size_t);
void __real_free(void *);
void *__wrap_malloc(size_t n)
{
	if (tracking)
		++calls;
	return __real_malloc(n);
}
void *__wrap_calloc(size_t n, size_t s)
{
	if (tracking)
		++calls;
	return __real_calloc(n, s);
}
void *__wrap_realloc(void *p, size_t n)
{
	if (tracking)
		++calls;
	return __real_realloc(p, n);
}
void __wrap_free(void *p)
{
	if (tracking)
		++calls;
	__real_free(p);
}
#endif
static hand_skin_rig s;
static hand_uv_map duplicate;
static uint64_t uvHash(void)
{
	GRE_RenderTarget t = YMGRE_Camera_GetRenderTarget(s.uvCamera);
	uint64_t h = 14695981039346656037ull;
	for (int i = 0; i < HAND_UV_SIZE * HAND_UV_SIZE; ++i)
	{
		GRErgb24 c = GRE_FramePixel_To_RGB24(t->data[i]);
		h = (h ^ c.R) * 1099511628211ull;
		h = (h ^ c.G) * 1099511628211ull;
		h = (h ^ c.B) * 1099511628211ull;
	}
	return h;
}
int main(void)
{
	CHECK(HandSkin_Init(&s));
	CHECK(HAND_POINTS == 1345 && HAND_FACES == 1698);
	printf("UV cache: %zu overlaps, %zu subpixel/uncovered patches\n", s.uv.overlapPixels,
		   s.uv.uncoveredPatches);
	CHECK(s.uv.overlapPixels < HAND_UV_SIZE * HAND_UV_SIZE / 100);
	CHECK(HandUV_Pick(&s.uv, -1, 0) == -1 && HandUV_Pick(&s.uv, HAND_UV_SIZE, 0) == -1);
	unsigned repeat[2] = {s.faces[0], s.faces[0]};
	CHECK(HandUV_Init(&duplicate, repeat, 2));
	CHECK(duplicate.overlapPixels > 0);
	int overlap = -1;
	for (int i = 0; i < HAND_UV_SIZE * HAND_UV_SIZE; ++i)
		if (duplicate.overlap[i])
		{
			overlap = i;
			break;
		}
	CHECK(overlap >= 0 &&
		  HandUV_Pick(&duplicate, overlap % HAND_UV_SIZE, overlap / HAND_UV_SIZE) == -2);
#ifdef WRAP_ALLOC
	tracking = 1;
#endif
	for (int i = 0; i < 80; ++i)
		CHECK(HandSkin_Step(&s));
	CHECK(s.reaction.contacts > 0 && s.normalLoad > 0 && s.touch.count == s.count);
	CHECK(YMPME_VecDot(YMPME_VecAdd(s.touch.wrench.force, s.reaction.probeWrench.force),
					   YMPME_VecAdd(s.touch.wrench.force, s.reaction.probeWrench.force)) < 1e-10);
	CHECK(YMPME_VecDot(YMPME_VecAdd(s.touch.wrench.torque, s.reaction.probeWrench.torque),
					   YMPME_VecAdd(s.touch.wrench.torque, s.reaction.probeWrench.torque)) < 1e-10);
	/* UV location follows the material coordinates, independent of viewing camera. */
	size_t strongest = 0;
	for (size_t i = 1; i < s.count; ++i)
		if (s.taxels[i].pressure > s.taxels[strongest].pressure)
			strongest = i;
	const uint16_t *tri = handTriangles[s.faces[strongest]];
	float u = 0, v = 0;
	for (int j = 0; j < 3; ++j)
	{
		u += handUV[tri[j]][0] / 3;
		v += handUV[tri[j]][1] / 3;
	}
	CHECK(HandUV_Pick(&s.uv, (int)(u * HAND_UV_SIZE), (int)(v * HAND_UV_SIZE)) == (int)strongest);
	HandSkin_Render(&s);
	uint64_t pressed = uvHash();
	Demo_ViewMove(&s.hand, .4f, .2f, 0);
	HandSkin_Render(&s);
	CHECK(uvHash() == pressed);
	/* Display mode must change the map, but cannot change measured physical loads. */
	pme_wrench savedWrench = s.touch.wrench;
	s.uvForce = 1;
	HandSkin_Render(&s);
	CHECK(uvHash() != pressed);
	CHECK(!memcmp(&savedWrench, &s.touch.wrench, sizeof savedWrench));
	s.uvForce = 0;
	HandSkin_Render(&s);
	CHECK(uvHash() == pressed);
	float load = s.normalLoad;
	s.indentation = .004f;
	for (int i = 0; i < 80; ++i)
		CHECK(HandSkin_Step(&s));
	CHECK(s.normalLoad > load);
	YMPME_ForceDevice_TareNext(&s.loadCell);
	CHECK(HandSkin_Step(&s) && fabsf(s.normalLoad) < 1e-6f);
	s.indentation = -.01f;
	for (int i = 0; i < 60; ++i)
		CHECK(HandSkin_Step(&s));
	CHECK(s.reaction.contacts == 0 && s.peakPressure == 0);
	HandSkin_Render(&s);
	CHECK(uvHash() != pressed);
	size_t cleared = 0;
	for (int y = 1; y < HAND_UV_SIZE - 1; ++y)
		for (int x = 1; x < HAND_UV_SIZE - 1; ++x)
		{
			size_t at = (size_t)y * HAND_UV_SIZE + x;
			int pick = HandUV_Pick(&s.uv, x, y);
			if (pick < 0 || s.uv.patch[at - 1] == HAND_UV_EMPTY ||
				s.uv.patch[at - HAND_UV_SIZE] == HAND_UV_EMPTY)
				continue;
			GRErgb24 c =
				GRE_FramePixel_To_RGB24(YMGRE_Camera_GetRenderTarget(s.uvCamera)->data[at]);
			CHECK(c.R == 35 && c.G == 100 && c.B == 200);
			++cleared;
		}
	CHECK(cleared > 1000);
	for (int region = 1; region <= 5; ++region)
	{
		s.indentation = .003f;
		CHECK(HandSkin_Select(&s, region));
		CHECK(HandSkin_Step(&s));
		CHECK(s.reaction.contacts > 0);
		CHECK(YMPME_VecDot(s.hand.joints[region * 3 - 1].feedback.reactionB.force,
						   s.hand.joints[region * 3 - 1].feedback.reactionB.force) > 1e-8f);
		for (int i = 0; i < 30; ++i)
			CHECK(HandSkin_Step(&s));
		for (int j = 0; j < HAND_JOINTS; ++j)
			CHECK(s.hand.joints[j].feedback.valid && isfinite(s.hand.joints[j].coordinate));
		HandSkin_Render(&s);
		CHECK(Demo_Validate(&s.hand));
	}
#ifdef WRAP_ALLOC
	tracking = 0;
	CHECK(calls == 0);
#endif
	HandSkin_Destroy(&s);
	puts("hand_low: six regions, load/depth, force-torque conservation, tare, release, physical "
		 "joints, UV picking/overlap/view invariance/Pa-N/release, no frame allocation passed");
	return 0;
}
