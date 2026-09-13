#include "rig.h"
#include "YMGRE_Coordinates_Transform.h"
#include <stdio.h>
#include <string.h>
#define CHECK(x)                                                                                   \
	do                                                                                             \
	{                                                                                              \
		if (!(x))                                                                                  \
		{                                                                                          \
			fprintf(stderr, "FAIL %d: %s\n", __LINE__, #x);                                        \
			return 1;                                                                              \
		}                                                                                          \
	} while (0)
#define V YMPME_Vec3
static float norm(pme_vec3 v) { return sqrtf(YMPME_VecDot(v, v)); }
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

static human_skin_rig s;
static uint64_t hash(GRE_RenderTarget t)
{
	uint64_t h = 14695981039346656037ull;
	for (int i = 0; i < t->width * t->height; i++)
	{
		GRErgb24 c = GRE_FramePixel_To_RGB24(t->data[i]);
		h = (h ^ c.R) * 1099511628211ull;
		h = (h ^ c.G) * 1099511628211ull;
		h = (h ^ c.B) * 1099511628211ull;
	}
	return h;
}
static int conservation(void)
{
	CHECK(norm(YMPME_VecAdd(s.touch.wrench.force, s.reaction.probeWrench.force)) < 1e-5f);
	CHECK(norm(YMPME_VecAdd(s.touch.wrench.torque, s.reaction.probeWrench.torque)) < 1e-5f);
	pme_wrench total = {0};
	for (unsigned b = 0; b < s.human.asset.info.boneCount; b++)
	{
		ske_mat34 m = s.human.poseBones[b].global;
		pme_vec3 origin = V(m.m[0][3] * .1f, m.m[1][3] * .1f, m.m[2][3] * .1f);
		total.force = YMPME_VecAdd(total.force, s.boneLoads[b].force);
		total.torque = YMPME_VecAdd(
			total.torque,
			YMPME_VecAdd(s.boneLoads[b].torque, YMPME_VecCross(YMPME_VecSub(origin, s.probe.center),
															   s.boneLoads[b].force)));
	}
	CHECK(norm(YMPME_VecSub(total.force, s.touch.wrench.force)) < 1e-4f);
	CHECK(norm(YMPME_VecSub(total.torque, s.touch.wrench.torque)) < 1e-4f);
	return 0;
}
int main(void)
{
	CHECK(HumanSkin_Init(&s));
	CHECK(YMSKE_MAX_INFLUENCES == 6 && s.human.asset.info.influences == 6);
	CHECK(s.human.asset.info.vertexCount == 7168 && s.human.asset.info.boneCount == 63 &&
		  s.count == 3584);
	/* At least one real vertex needs slots five/six; do not silently trim the asset. */
	size_t six = 0;
	for (unsigned v = 0; v < s.human.asset.info.vertexCount; v++)
		if (s.human.vertices[v].weights.weight[4] > 0 || s.human.vertices[v].weights.weight[5] > 0)
			six++;
	CHECK(six > 0);
	CHECK(s.autoUv.overlapPixels == 0 && s.autoUv.uncoveredPatches < 20);
#ifdef WRAP_ALLOC
	tracking = 1;
#endif
	for (int region = 0; region < 9; region++)
	{
		CHECK(HumanSkin_Region(&s, region));
		for (int i = 0; i < 12; i++)
			CHECK(HumanSkin_Step(&s));
		CHECK(s.reaction.contacts > 0 && s.totalNormal > 0 &&
			  s.taxels[s.probePatch].normalForce > 0);
		CHECK(norm(s.boneLoads[s.loadBone].force) > 0);
		CHECK(!conservation());
	}
	CHECK(HumanSkin_Region(&s, 0));
	CHECK(HumanSkin_Step(&s));
	float load = s.totalNormal;
	s.indentation = .008f;
	for (int i = 0; i < 12; i++)
		CHECK(HumanSkin_Step(&s));
	CHECK(s.totalNormal > load);
	pme_vec3 force = s.forces[s.probePatch];
	s.offsetU = .002f;
	CHECK(HumanSkin_Step(&s));
	CHECK(YMPME_VecDot(s.taxels[s.probePatch].shear, s.tangentU) > 0);
	s.offsetU = 0;
	CHECK(HumanSkin_Step(&s));
	CHECK(YMPME_VecDot(s.taxels[s.probePatch].shear, s.tangentU) < 0);
	s.offsetV = .002f;
	CHECK(HumanSkin_Step(&s));
	CHECK(YMPME_VecDot(s.taxels[s.probePatch].shear, s.tangentV) > 0);
	s.offsetV = 0;
	CHECK(HumanSkin_Step(&s));
	CHECK(YMPME_VecDot(s.taxels[s.probePatch].shear, s.tangentV) < 0);
	CHECK(norm(YMPME_VecSub(force, s.forces[s.probePatch])) > 0);
	for (int i = 0; i < 8; i++)
		CHECK(HumanSkin_Step(&s));
	YMPME_ForceDevice_TareNext(&s.loadCell);
	CHECK(HumanSkin_Step(&s));
	CHECK(norm(s.loadReading.sample.wrench.force) < 1e-6f && s.totalNormal > 0);
	HumanSkin_Render(&s);
	uint64_t uv = hash(YMGRE_Camera_GetRenderTarget(s.uvCamera));
	Demo_ViewMove(&s.human, .2f, .1f, 0);
	HumanSkin_Render(&s);
	CHECK(hash(YMGRE_Camera_GetRenderTarget(s.uvCamera)) == uv);
	pme_vec3 center = s.probe.center;
	pme_wrench wrench = s.touch.wrench;
	s.contactView.overview = 2;
	s.source = 1;
	HumanSkin_Render(&s);
	CHECK(hash(YMGRE_Camera_GetRenderTarget(s.uvCamera)) != uv);
	CHECK(!memcmp(&center, &s.probe.center, sizeof center) &&
		  !memcmp(&wrench, &s.touch.wrench, sizeof wrench));
	s.source = 0;
	s.contactView.overview = 3;
	HumanSkin_Render(&s);
	CHECK(hash(YMGRE_Camera_GetRenderTarget(s.uvCamera)) == uv);
	/* A UV zoom is just a viewport transform; picking at the zoom anchor remains the same. */
	int pixel = -1;
	for (int y = 0; y < UV_VIEW && pixel < 0; y++)
		for (int x = 0; x < UV_VIEW; x++)
		{
			int p = HumanSkin_PickUV(&s, x, y);
			if (p == s.probePatch)
			{
				pixel = y * UV_VIEW + x;
				break;
			}
		}
	CHECK(pixel >= 0);
	int x = pixel % UV_VIEW, y = pixel / UV_VIEW, expected = HumanSkin_PickUV(&s, x, y);
	HumanSkin_ZoomUV(&s, x, y, 2);
	CHECK(s.uvZoom == 2 && HumanSkin_PickUV(&s, x, y) == expected);
	HumanSkin_Render(&s);
	CHECK(hash(YMGRE_Camera_GetRenderTarget(s.uvCamera)) != uv);
	s.uvZoom = 1;
	s.uvCenterU = s.uvCenterV = .5f;
	/* Test depth-aware 3D picking on the visible front-side pressure patch. */
	CHECK(HumanSkin_Region(&s, 0));
	CHECK(HumanSkin_Step(&s));
	HumanSkin_Render(&s);
	pme_vec3 pos = s.patches[s.probePatch].position;
	gre_fvector4d p = {pos.x * 10, pos.y * 10, pos.z * 10, 1}, cam;
	YMGRE_Point_WorldToCamera(&p, &cam, &s.human.camera->move.TMat);
	YMGRE_Point_CameraToViewPlane(&cam, s.human.camera->perspectPlane.Dis);
	YMGRE_Point_ViewPlaneToWindows(&cam, s.human.camera);
	CHECK(HumanSkin_Pick3D(&s, cam.x, cam.y) == s.probePatch);
	CHECK(HumanSkin_Pick3D(&s, -100, -100) == -1);
	s.motion = 1;
	s.rub = 1;
	for (int i = 0; i < 240; i++)
	{
		CHECK(HumanSkin_Step(&s));
		CHECK(!conservation());
		CHECK(isfinite(s.totalNormal));
	}
	CHECK(s.reaction.contacts > 0);
	HumanSkin_Render(&s);
	s.motion = s.rub = 0;
	s.indentation = -.12f;
	for (int i = 0; i < 12; i++)
		CHECK(HumanSkin_Step(&s));
	CHECK(s.reaction.contacts == 0 && s.totalNormal == 0 && s.peakPressure == 0);
	HumanSkin_Render(&s);
	for (size_t p2 = 0; p2 < s.count; p2++)
		CHECK(s.colors[p2].R == 0 && s.colors[p2].G == 128 && s.colors[p2].B == 128);
#ifdef WRAP_ALLOC
	tracking = 0;
	CHECK(calls == 0);
#endif
	HumanSkin_Destroy(&s);
	puts("human skin: six weights, nine regions, compression/shear, full wrench conservation, "
		 "tare/release, GRE UV/zoom/pick, animation feedback, no frame allocation PASS");
	return 0;
}
