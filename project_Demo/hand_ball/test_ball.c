#include "ball.h"
#include "force_view.h"
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
static float norm(pme_vec3 x) { return sqrtf(YMPME_VecDot(x, x)); }
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

static hand_ball_scene s;
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
static int basisTest(void)
{
	pme_vec3 p[3] = {V(0, 0, 0), V(1, 0, 0), V(0, 1, 0)}, local;
	float uv[6] = {0, 0, 1, 0, 0, 1};
	pme_vec3 directions[5] = {V(0, 0, -1), V(.3f, 0, -1), V(-.3f, 0, -1), V(0, .3f, -1),
							  V(0, -.3f, -1)};
	for (int i = 0; i < 5; i++)
	{
		CHECK(HandForce_Local(p, uv, V(0, 0, 1), directions[i], &local));
		CHECK(fabsf(local.x - directions[i].x) < 1e-6f &&
			  fabsf(local.y - directions[i].y) < 1e-6f && fabsf(local.z - 1) < 1e-6f);
		CHECK(fabsf(norm(local) - norm(directions[i])) < 1e-6f);
	}
	uv[5] = -1;
	CHECK(HandForce_Local(p, uv, V(0, 0, 1), V(.2f, .3f, -1), &local));
	CHECK(fabsf(local.x - .2f) < 1e-6f && fabsf(local.y + .3f) < 1e-6f);
	uv[5] = 1;
	pme_quaternion q;
	CHECK(YMPME_Quaternion_AxisAngle(V(1, 2, 3), .8f, &q) == PME_OK);
	pme_rotation rotation = YMPME_Quaternion_Rotation(q);
	for (int i = 0; i < 3; i++)
		p[i] = YMPME_Rotation_Apply(rotation, p[i]);
	CHECK(HandForce_Local(p, uv, YMPME_Rotation_Apply(rotation, V(0, 0, 1)),
						  YMPME_Rotation_Apply(rotation, V(.2f, .3f, -1)), &local));
	CHECK(norm(YMPME_VecSub(local, V(.2f, .3f, 1))) < 1e-5f);
	pme_vec3 old = local;
	uv[5] = 0;
	CHECK(!HandForce_Local(p, uv, V(0, 0, 1), V(0, 0, -1), &local));
	CHECK(!memcmp(&old, &local, sizeof old));
	return 0;
}
int main(void)
{
	CHECK(!basisTest());
	int clipped = 0;
	GRErgb24 zero = HandForce_RGB(V(0, 0, 0), &clipped);
	CHECK(zero.R == 0 && zero.G == 128 && zero.B == 128 && !clipped);
	GRErgb24 positive = HandForce_RGB(V(.05f, .05f, .1f), &clipped);
	CHECK(positive.R == 255 && positive.G == 255 && positive.B == 255 && !clipped);
	GRErgb24 negative = HandForce_RGB(V(-.05f, -.05f, 0), &clipped);
	CHECK(negative.R == 0 && negative.G == 0 && negative.B == 0 && !clipped);
	GRErgb24 onlyU = HandForce_RGB(V(.05f, 0, 0), NULL),
			 onlyV = HandForce_RGB(V(0, -.05f, 0), NULL), onlyN = HandForce_RGB(V(0, 0, .1f), NULL);
	CHECK(onlyU.R == 0 && onlyU.G == 255 && onlyU.B == 128);
	CHECK(onlyV.R == 0 && onlyV.G == 128 && onlyV.B == 0);
	CHECK(onlyN.R == 255 && onlyN.G == 128 && onlyN.B == 128);
	GRErgb24 limited = HandForce_RGB(V(.1f, -.1f, .2f), &clipped);
	CHECK(limited.R == 255 && limited.G == 255 && limited.B == 0 && clipped);
	CHECK(HandBall_Init(&s));
	CHECK(s.autoCharts > 0 && s.autoUv.overlapPixels == 0 && s.autoUv.uncoveredPatches == 0);
	CHECK(s.skin.hand.object->pointNum == HAND_POINTS &&
		  s.skin.hand.object->polygonNum == HAND_FACES);
	/* Init UV matches face identity and conserves geometric data/bone weights. */
	size_t picks = 0;
	for (size_t i = 0; i < s.skin.count; i++)
	{
		float u = 0, v = 0;
		for (int j = 0; j < 3; j++)
		{
			u += s.autoCorners[i * 6 + j * 2] / 3;
			v += s.autoCorners[i * 6 + j * 2 + 1] / 3;
		}
		int pick = HandUV_Pick(&s.autoUv, (int)(u * HAND_UV_SIZE), (int)(v * HAND_UV_SIZE));
		if (pick == (int)i)
			picks++;
	}
	CHECK(picks > s.skin.count * 9 / 10);
#ifdef WRAP_ALLOC
	tracking = 1;
#endif
	pme_vec3 start = s.ball.position;
	for (int i = 0; i < 480; i++)
	{
		CHECK(HandBall_Step(&s));
		CHECK(norm(YMPME_VecAdd(s.skin.touch.wrench.force, s.skin.reaction.probeWrench.force)) <
			  1e-5f);
		CHECK(norm(YMPME_VecAdd(s.skin.touch.wrench.torque, s.skin.reaction.probeWrench.torque)) <
			  1e-6f);
	}
	CHECK(s.ball.position.z < start.z - .008f && s.ball.position.z > .025f);
	CHECK(s.skin.reaction.contacts > 10 && s.totalNormal > .3f && s.floorForce == 0 &&
		  norm(s.ball.velocity) < .002f);
	s.arrows = 1;
	HandBall_Render(&s);
	CHECK(s.arrowCount > 0);
	s.arrows = 0;
	HandBall_Render(&s);
	size_t checked = 0;
	for (int y = 1; y < HAND_UV_SIZE - 1; y++)
		for (int x = 1; x < HAND_UV_SIZE - 1; x++)
		{
			size_t at = (size_t)y * HAND_UV_SIZE + x;
			int p = s.autoUv.patch[at];
			if (p == HAND_UV_EMPTY || s.autoUv.patch[at - 1] == HAND_UV_EMPTY ||
				s.autoUv.patch[at - HAND_UV_SIZE] == HAND_UV_EMPTY)
				continue;
			GRErgb24 c =
				GRE_FramePixel_To_RGB24(YMGRE_Camera_GetRenderTarget(s.skin.uvCamera)->data[at]);
			CHECK(c.R == s.rgbPatch[p].R && c.G == s.rgbPatch[p].G && c.B == s.rgbPatch[p].B);
			checked++;
		}
	CHECK(checked > 10000);
	s.arrows = 1;
	HandBall_Render(&s);
	pme_rigid_body saved = s.ball;
	uint64_t solid = hash(YMGRE_Camera_GetRenderTarget(s.skin.hand.camera));
	uint64_t uv = hash(YMGRE_Camera_GetRenderTarget(s.skin.uvCamera));
	s.wire = 1;
	HandBall_Render(&s);
	CHECK(hash(YMGRE_Camera_GetRenderTarget(s.skin.hand.camera)) != solid);
	CHECK(hash(YMGRE_Camera_GetRenderTarget(s.skin.uvCamera)) == uv);
	CHECK(!memcmp(&saved, &s.ball, sizeof saved));
	Demo_ViewMove(&s.skin.hand, .1f, .1f, 0);
	HandBall_Render(&s);
	CHECK(hash(YMGRE_Camera_GetRenderTarget(s.skin.uvCamera)) == uv);
	s.sourceUv = 1;
	HandBall_Render(&s);
	CHECK(hash(YMGRE_Camera_GetRenderTarget(s.skin.uvCamera)) != uv);
	CHECK(!memcmp(&saved, &s.ball, sizeof saved));
	s.sourceUv = 0;
	s.arrows = 0;
	HandBall_Render(&s);
	CHECK(hash(YMGRE_Camera_GetRenderTarget(s.skin.uvCamera)) != uv);
	s.arrows = 1;
	YMPME_Rigid_Impulse(&s.ball, V(0, 0, 0), V(0, 0, .00002f));
	float spin = norm(s.ball.angularVelocity);
	for (int i = 0; i < 240; i++)
		CHECK(HandBall_Step(&s));
	CHECK(norm(s.ball.angularVelocity) < spin * .5f);
	s.motion = 1;
	size_t contacting = 0;
	float minX = s.ball.position.x, maxX = minX;
	for (int i = 0; i < 1440; i++)
	{
		CHECK(HandBall_Step(&s));
		if (s.skin.reaction.contacts)
			contacting++;
		minX = fminf(minX, s.ball.position.x);
		maxX = fmaxf(maxX, s.ball.position.x);
		CHECK(!s.lost && norm(s.ball.velocity) < 1 && norm(s.ball.angularVelocity) < 30);
	}
	CHECK(contacting > 1400 && maxX - minX > .025f);
	/* 打开并倾斜后，球能离手落桌，绝不吸附手掌/自动重置球。 */
	s.motion = 0;
	s.grip = 0;
	s.tilt = 1.1f;
	for (int i = 0; i < 480; i++)
		CHECK(HandBall_Step(&s));
	CHECK(s.skin.reaction.contacts == 0 && s.floorForce > .3f && s.lost);
	CHECK(fabsf(s.ball.position.z - (s.floorZ + s.radius)) < .002f);
	/* 未接触时必须是自由刚体，单步重力精确变化。 */
	CHECK(HandBall_Reset(&s));
	s.ball.position = V(1, 1, 1);
	s.ball.velocity = V(.2f, .1f, 0);
	CHECK(HandBall_Step(&s));
	CHECK(s.skin.reaction.contacts == 0 && s.floorForce == 0);
	CHECK(fabsf(s.ball.velocity.z + 9.81f / 240) < 1e-6f && fabsf(s.ball.velocity.x - .2f) < 1e-6f);
	CHECK(HandBall_Reset(&s));
	CHECK(norm(YMPME_VecSub(s.ball.position, start)) < 1e-6f);
#ifdef WRAP_ALLOC
	tracking = 0;
	CHECK(calls == 0);
#endif
	HandBall_Destroy(&s);
	puts("hand ball: free-body gravity, grip/move/release, equal-opposite loads, spin friction, "
		 "wire/UV invariance, GRE unwrap, four-way force axes, no frame allocations PASS");
	return 0;
}
