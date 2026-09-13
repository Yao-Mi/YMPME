#include "rig.h"
#include "observation.h"
#include <stdio.h>
#include <string.h>
#include <time.h>
#define V YMPME_Vec3
#define CHECK(x)                                                                                   \
	do                                                                                             \
	{                                                                                              \
		if (!(x))                                                                                  \
		{                                                                                          \
			fprintf(stderr, "human self FAIL %d: %s\n", __LINE__, #x);                             \
			goto fail;                                                                             \
		}                                                                                          \
	} while (0)
static human_skin_rig s;
static human_self_touch before;
static uint8_t gridBefore[64 * 64 * 3], gridAfter[64 * 64 * 3];
static pme_vec3 local[DEMO_FACES];
static pme_vec3 previousPoints[DEMO_FACES];
static float norm(pme_vec3 v) { return sqrtf(YMPME_VecDot(v, v)); }
#ifdef WRAP_ALLOC
static int tracking;
static size_t allocations;
void *__real_malloc(size_t);
void *__real_calloc(size_t, size_t);
void *__real_realloc(void *, size_t);
void __real_free(void *);
void *__wrap_malloc(size_t n)
{
	if (tracking)
		allocations++;
	return __real_malloc(n);
}
void *__wrap_calloc(size_t n, size_t z)
{
	if (tracking)
		allocations++;
	return __real_calloc(n, z);
}
void *__wrap_realloc(void *p, size_t n)
{
	if (tracking)
		allocations++;
	return __real_realloc(p, n);
}
void __wrap_free(void *p)
{
	if (tracking)
		allocations++;
	__real_free(p);
}
#endif
int main(void)
{
	setbuf(stdout, NULL);
	CHECK(HumanSkin_Init(&s));
	CHECK(HumanSelf_Enable(&s, 1));
	int contactFrames = 0, releaseFrames = 0, handRGB = 0, bodyRGB = 0;
	float peak = 0, deepest = 0;
	double elapsed = 0;
	for (int i = 0; i < 1200; i++)
	{
		clock_t begin = clock();
		CHECK(HumanSkin_Step(&s));
		elapsed += (double)(clock() - begin) / CLOCKS_PER_SEC;
		CHECK(s.self.ikError < .001f);
		CHECK(s.self.result.maxDepth < .0042f);
		deepest = fmaxf(deepest, s.self.result.maxDepth);
		peak = fmaxf(peak, s.self.handNormal);
		if (i > 250 && i < 650)
		{
			CHECK(s.self.handNormal > 0 && s.self.bodyNormal > 0);
			contactFrames++;
			CHECK(s.self.correction > .001f);
			CHECK(norm(s.touch.wrench.force) < 2e-5f);
			CHECK(norm(s.touch.wrench.torque) < 2e-5f);
			pme_vec3 totalF = V(0, 0, 0), totalT = V(0, 0, 0);
			for (unsigned b = 0; b < s.human.asset.info.boneCount; b++)
			{
				ske_mat34 m = s.human.poseBones[b].global;
				pme_vec3 p = V(m.m[0][3] * .1f, m.m[1][3] * .1f, m.m[2][3] * .1f);
				totalF = YMPME_VecAdd(totalF, s.boneLoads[b].force);
				totalT =
					YMPME_VecAdd(totalT, YMPME_VecAdd(s.boneLoads[b].torque,
													  YMPME_VecCross(p, s.boneLoads[b].force)));
			}
			CHECK(norm(totalF) < 2e-5f && norm(totalT) < 2e-5f);
		}
		if (i > 1000)
		{
			CHECK(s.self.result.contacts == 0 && s.totalNormal == 0);
			releaseFrames++;
		}
		if (i == 420)
		{
			HumanSkin_Render(&s);
			for (size_t p = 0; p < s.count; p++)
				if (s.colors[p].R > 0)
				{
					handRGB += s.self.faces[p].region == 1;
					bodyRGB += s.self.faces[p].region == 2;
				}
			CHECK(handRGB > 0 && bodyRGB > 0);
			pme_sensor_stamp stamp;
			CHECK(HumanSkin_ObserveGrid(&s, gridBefore, sizeof gridBefore, local, DEMO_FACES,
										&stamp));
			int marked[3] = {0};
			GRE_RenderTarget grid = YMGRE_Camera_GetRenderTarget(s.uvCamera);
			for (size_t p = 0; p < s.count; p++)
				if (HumanSkin_IsContactMarked(&s, p))
				{
					marked[s.self.faces[p].region]++;
					uint32_t c = s.contactView.faceCells[p];
					size_t pixel = (c / 64 * 8) * UV_VIEW + c % 64 * 8;
					GRE_FramePixel expected = GRE_FramePixel_From_RGB24((GRErgb24){245, 245, 245});
					CHECK(!memcmp(&grid->data[pixel], &expected, sizeof expected));
				}
			CHECK(marked[1] > 0 && marked[2] > 0);
			s.contactMarks = 0;
			HumanSkin_Render(&s);
			CHECK(
				HumanSkin_ObserveGrid(&s, gridAfter, sizeof gridAfter, local, DEMO_FACES, &stamp));
			CHECK(!memcmp(gridBefore, gridAfter, sizeof gridBefore));
			s.contactMarks = 1;
			HumanSkin_Render(&s);
			CHECK(s.contactView.density.overlapPixels == 0 &&
				  s.contactView.density.uncoveredPatches == 0);
			float low = 1, high = 0;
			for (size_t p = 0; p < s.count; p++)
			{
				const float *uv = s.contactView.densityCorners + p * 6;
				float area =
					fabsf((uv[2] - uv[0]) * (uv[5] - uv[1]) - (uv[4] - uv[0]) * (uv[3] - uv[1])) *
					.5f;
				low = fminf(low, area);
				high = fmaxf(high, area);
			}
			CHECK(high / low < 1.0002f);

			CHECK(Demo_Save(&s.human, "/tmp/ympme-self-contact-test.ppm"));
		}
	}
	CHECK(contactFrames > 300 && releaseFrames > 100);
	/* Manual press and collision-filter exclusion, then re-enable protection. */
	s.self.cycle = 0;
	s.rub = 0;
	s.indentation = .005f;
	CHECK(HumanSkin_Step(&s));
	CHECK(s.self.result.contacts > 0);
	s.self.filtered = 1;
	CHECK(HumanSkin_Step(&s));
	CHECK(s.self.result.contacts == 0 && s.totalNormal == 0 && s.self.correction == 0);
	s.self.filtered = 0;
	CHECK(HumanSkin_Step(&s));
	CHECK(HumanSkin_Step(&s));
	CHECK(s.self.result.contacts > 0 && s.self.result.maxDepth < .0042f);
	/* Replay an identical physical step with different display settings. */
	before = s.self;
	pme_force_device beforeDevice = s.loadCell;
	memcpy(previousPoints, s.previousPoint, sizeof previousPoints);
	double beforeTime = s.time;
	CHECK(HumanSkin_Step(&s));
	pme_vec3 expected = s.self.result.sourceWrench.force;
	float z = s.self.corrected.z;
	s.self = before;
	s.loadCell = beforeDevice;
	memcpy(s.previousPoint, previousPoints, sizeof previousPoints);
	s.time = beforeTime;
	s.source = !s.source;
	s.colorMode = 2;
	Demo_ViewMove(&s.human, .3f, .1f, 0);
	CHECK(HumanSkin_Step(&s));
	CHECK(norm(YMPME_VecSub(expected, s.self.result.sourceWrench.force)) < 1e-6f &&
		  fabsf(z - s.self.corrected.z) < 1e-6f);
	YMPME_ForceDevice_TareNext(&s.loadCell);
	for (int i = 0; i < 60; i++)
		CHECK(HumanSkin_Step(&s));
	CHECK(norm(s.loadReading.sample.wrench.force) < .001f);
#ifdef WRAP_ALLOC
	tracking = 1;
#endif
	CHECK(HumanSkin_Step(&s));
	HumanSkin_Render(&s);
#ifdef WRAP_ALLOC
	tracking = 0;
	CHECK(allocations == 0);
#endif
	s.indentation = -.12f;
	for (int i = 0; i < 10; i++)
		CHECK(HumanSkin_Step(&s));
	CHECK(s.totalNormal == 0);
	CHECK(HumanSelf_Enable(&s, 0));
	s.indentation = .005f;
	CHECK(HumanSkin_Step(&s));
	CHECK(s.reaction.contacts > 0);
	printf("HUMAN_SELF PASS: %d contact frames, %d released, RGB hand/body %d/%d, peak %.3f N, "
		   "shell depth %.3f mm, CPU %.2f ms/step\n",
		   contactFrames, releaseFrames, handRGB, bodyRGB, peak, deepest * 1000,
		   elapsed / 1200 * 1000);
	HumanSkin_Destroy(&s);
	return 0;
fail:
#ifdef WRAP_ALLOC
	tracking = 0;
#endif
	HumanSkin_Destroy(&s);
	return 1;
}
