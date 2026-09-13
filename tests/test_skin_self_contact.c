#include "MODEL/SKIN/YMPME_SkinSelfContact.h"
#include "CORE/YMPME_MathBase.h"
#include <stdio.h>
#include <string.h>
#define V YMPME_Vec3
#define CHECK(x)                                                                                   \
	do                                                                                             \
	{                                                                                              \
		if (!(x))                                                                                  \
		{                                                                                          \
			fprintf(stderr, "self contact FAIL %d: %s\n", __LINE__, #x);                           \
			return 1;                                                                              \
		}                                                                                          \
	} while (0)
static float norm(pme_vec3 v) { return sqrtf(YMPME_VecDot(v, v)); }
static pme_vec3 turn(pme_vec3 v) { return V(v.z, v.x, v.y); }
int main(void)
{
	pme_skin_face f[2] = {0};
	pme_vec3 forces[2], moments[2];
	pme_skin_self_target scratch[2];
	pme_skin_self_result r;
	f[0] = (pme_skin_face){{{-.01f, -.01f, .004f}, {.01f, -.01f, .004f}, {0, .02f, .004f}},
						   {{0, 0, .004f}, {0, 0, -1}, {0, 0, 0}, .0003f},
						   1};
	f[1] = (pme_skin_face){
		{{-1, -1, 0}, {1, -1, 0}, {0, 2, 0}}, {{0, 0, 0}, {0, 0, 1}, {0, 0, 0}, 3}, 2};
	pme_skin_self_desc d = YMPME_SkinSelfContact_DefaultDesc();
	d.sourceRegions = 1u << 1;
	d.targetRegions = 1u << 2;
	d.allow[1] = 1u << 2;
	d.allow[2] = 1u << 1;
#define SOLVE() YMPME_SkinSelfContact_Solve(&d, f, 2, V(0, 0, 0), forces, moments, scratch, 2, &r)
	CHECK(SOLVE() == PME_OK);
	CHECK(r.contacts == 4);
	CHECK(fabsf(forces[0].z - .24f) < 1e-5f);
	CHECK(norm(YMPME_VecAdd(forces[0], forces[1])) < 1e-7f);
	d.material.tangentDamping = 10000;
	f[0].surface.velocity = V(1, 0, 0);
	CHECK(SOLVE() == PME_OK);
	CHECK(forces[0].x < 0);
	CHECK(fabsf(forces[0].x) <= d.material.friction * forces[0].z + 1e-7f);
	pme_vec3 torque = YMPME_Vec3(0, 0, 0);
	for (int i = 0; i < 2; i++)
		torque = YMPME_VecAdd(
			torque, YMPME_VecAdd(moments[i], YMPME_VecCross(f[i].surface.position, forces[i])));
	CHECK(norm(torque) < 1e-7f);
	pme_vec3 original = forces[0];
	for (int i = 0; i < 2; i++)
	{
		for (int k = 0; k < 3; k++)
			f[i].vertex[k] = turn(f[i].vertex[k]);
		f[i].surface.position = turn(f[i].surface.position);
		f[i].surface.normal = turn(f[i].surface.normal);
		f[i].surface.velocity = turn(f[i].surface.velocity);
	}
	CHECK(SOLVE() == PME_OK);
	CHECK(norm(YMPME_VecSub(forces[0], turn(original))) < 1e-6f);
	f[0].surface.velocity = V(0, -1, 0);
	CHECK(SOLVE() == PME_OK);
	CHECK(forces[0].y > 0);
	d.allow[2] = 0;
	CHECK(SOLVE() == PME_OK);
	CHECK(r.contacts == 0 && r.candidates == 0);
	CHECK(norm(forces[0]) == 0);
	d.allow[2] = 1u << 1;
	d.material.maxPressure = 50;
	CHECK(SOLVE() == PME_OK);
	CHECK(r.saturated);
	CHECK(fabsf(forces[0].x - .015f) < 1e-6f);
	for (int k = 0; k < 3; k++)
		f[0].vertex[k].x += .2f;
	f[0].surface.position.x += .2f;
	CHECK(SOLVE() == PME_OK);
	CHECK(r.contacts == 0 && norm(forces[0]) == 0);
	memset(forces, 0x5a, sizeof forces);
	pme_vec3 saved[2];
	memcpy(saved, forces, sizeof saved);
	CHECK(YMPME_SkinSelfContact_Solve(&d, f, 2, V(0, 0, 0), forces, moments, scratch, 1, &r) ==
		  PME_CAPACITY_EXCEEDED);
	CHECK(memcmp(saved, forces, sizeof saved) == 0);
	d.sourceRegions = d.targetRegions;
	CHECK(SOLVE() == PME_INVALID_ARGUMENT);
	CHECK(memcmp(saved, forces, sizeof saved) == 0);
	d.sourceRegions = 1u << 1;
	f[0].region = 32;
	CHECK(SOLVE() == PME_INVALID_ARGUMENT);
	CHECK(memcmp(saved, forces, sizeof saved) == 0);
	f[0].region = 1;
	f[0].vertex[1] = f[0].vertex[0];
	CHECK(SOLVE() == PME_INVALID_ARGUMENT);
	CHECK(memcmp(saved, forces, sizeof saved) == 0);
	puts("SKIN_SELF_CONTACT PASS: filters, shell, friction, rotation, force/torque, release, "
		 "validation");
	return 0;
}
