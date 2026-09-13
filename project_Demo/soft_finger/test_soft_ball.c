#include "SOFTBODY/YMPME_SoftBall.h"
#include <stdio.h>
#include <string.h>
#define CHECK(x)                                                                                   \
	do                                                                                             \
	{                                                                                              \
		if (!(x))                                                                                  \
		{                                                                                          \
			fprintf(stderr, "line %d: %s\n", __LINE__, #x);                                        \
			return 1;                                                                              \
		}                                                                                          \
	} while (0)
int main(void)
{
	pme_soft_ball b, firm;
	CHECK(YMPME_SoftBall_Init(&b, .22f, 40, 14) == PME_OK);
	CHECK(YMPME_SoftBall_Init(&firm, .22f, 600, 14) == PME_OK);
	for (int i = 0; i < 240 * 4; ++i)
	{
		CHECK(YMPME_SoftBall_Step(&b, 4, 1.f / 240) == PME_OK);
		CHECK(YMPME_SoftBall_Step(&firm, 4, 1.f / 240) == PME_OK);
		pme_vec3 scale = YMPME_SoftBall_Scale(&b);
		CHECK(fabsf(scale.x * scale.y * scale.z - 1) < 2e-6f);
	}
	CHECK(fabsf(b.compression - .1f) < 1e-5f);
	CHECK(firm.compression < b.compression * .1f);
	pme_vec3 original = YMPME_Vec3(.11f, .08f, .17f), scale = YMPME_SoftBall_Scale(&b);
	pme_vec3 point = YMPME_Vec3(original.x * scale.x, original.y * scale.y, original.z * scale.z);
	pme_vec3 gradient = YMPME_SoftBall_Mode(&b, point);
	pme_soft_ball nearby = b;
	nearby.compression += 1e-4f;
	pme_vec3 next = YMPME_SoftBall_Scale(&nearby);
	CHECK(fabsf((next.x - scale.x) * original.x / 1e-4f - gradient.x) < .002f);
	CHECK(fabsf((next.y - scale.y) * original.y / 1e-4f - gradient.y) < .002f);
	for (int i = 0; i < 240 * 4; ++i)
	{
		float q = b.compression;
		CHECK(YMPME_SoftBall_Step(&b, 0, 1.f / 240) == PME_OK);
		CHECK(b.compression >= 0 && b.compression <= q);
	}
	CHECK(b.compression < 2e-6f);
	pme_soft_ball before = b;
	CHECK(YMPME_SoftBall_Step(&b, NAN, .004f) == PME_INVALID_ARGUMENT);
	CHECK(!memcmp(&b, &before, sizeof b));
	CHECK(YMPME_SoftBall_Step(&b, 1, 0) == PME_INVALID_ARGUMENT);
	CHECK(!memcmp(&b, &before, sizeof b));
	CHECK(YMPME_SoftBall_Step(&b, 1e6f, .004f) == PME_OK);
	CHECK(b.compression == .55f * b.radius);
	CHECK(YMPME_SoftBall_Step(&b, -1e6f, .004f) == PME_OK);
	CHECK(b.compression == -.25f * b.radius);
	puts("SOFT_BALL PASS (volume, load response, recovery, mode derivative, bounds)");
	return 0;
}
