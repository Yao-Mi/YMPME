#include "MODEL/SKIN/YMPME_SkinContact.h"
#include "CORE/YMPME_MathBase.h"
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
int main(void)
{
	pme_surface_patch p[2] = {{{0, 0, 0}, {0, 0, 1}, {0, 0, 0}, .01f},
							  {{1, 0, 0}, {0, 0, 1}, {0, 0, 0}, .01f}};
	pme_skin_probe sphere = {{0, 0, .09f}, {0, 0, 0}, .1f};
	pme_skin_contact_desc d = YMPME_SkinContact_DefaultDesc();
	d.stiffness = 1000;
	d.damping = 0;
	pme_vec3 f[2], saved[2];
	pme_skin_contact_result result, old;
	CHECK(YMPME_SkinContact_Sphere(&d, p, 2, &sphere, f, 2, &result) == PME_OK);
	CHECK(NEAR(f[0].z, -.1) && f[1].z == 0 && result.contacts == 1 &&
		  NEAR(result.probeWrench.force.z, .1));
	d.stiffness *= 2;
	CHECK(YMPME_SkinContact_Sphere(&d, p, 2, &sphere, f, 2, &result) == PME_OK &&
		  NEAR(f[0].z, -.2));
	sphere.velocity.x = 1;
	d.tangentDamping = 100;
	d.friction = .3f;
	CHECK(YMPME_SkinContact_Sphere(&d, p, 2, &sphere, f, 2, &result) == PME_OK &&
		  NEAR(f[0].x, .06));
	CHECK(NEAR(result.probeWrench.torque.y, .0054));
	d.maxPressure = 5;
	CHECK(YMPME_SkinContact_Sphere(&d, p, 2, &sphere, f, 2, &result) == PME_OK &&
		  result.saturated && NEAR(f[0].z, -.05));
	sphere.center.z = .2f;
	CHECK(YMPME_SkinContact_Sphere(&d, p, 2, &sphere, f, 2, &result) == PME_OK && !result.contacts);
	sphere.center.z = -.09f;
	CHECK(YMPME_SkinContact_Sphere(&d, p, 2, &sphere, f, 2, &result) == PME_OK && !result.contacts);
	sphere.center.z = .09f;
	sphere.velocity = (pme_vec3){0, 0, -.01f};
	d.stiffness = 0;
	d.damping = 100;
	d.maxPressure = 5000;
	CHECK(YMPME_SkinContact_Sphere(&d, p, 2, &sphere, f, 2, &result) == PME_OK &&
		  NEAR(f[0].z, -.01));
	sphere.velocity.z = .01f;
	CHECK(YMPME_SkinContact_Sphere(&d, p, 2, &sphere, f, 2, &result) == PME_OK && !result.contacts);
	memcpy(saved, f, sizeof f);
	old = result;
	p[1].normal.z = 2;
	CHECK(YMPME_SkinContact_Sphere(&d, p, 2, &sphere, f, 2, &result) == PME_INVALID_ARGUMENT);
	CHECK(!memcmp(f, saved, sizeof f) && !memcmp(&old, &result, sizeof result));
	CHECK(YMPME_SkinContact_Sphere(&d, p, 2, &sphere, f, 1, &result) == PME_CAPACITY_EXCEEDED);

	/* 自由球：纯径向压力不产生转矩，自旋摩擦反向做功。 */
	p[0] = (pme_surface_patch){{.03f, 0, 0}, {0, 0, 1}, {0, 0, 0}, .01f};
	sphere = (pme_skin_probe){{0, 0, .09f}, {0, 0, 0}, .1f};
	d = YMPME_SkinContact_DefaultDesc();
	d.friction = 0;
	CHECK(YMPME_SkinContact_SphereRigid(&d, p, 1, &sphere, YMPME_Vec3(0, 0, 2), f, 2, &result) ==
		  PME_OK);
	CHECK(YMPME_VecDot(result.probeWrench.torque, result.probeWrench.torque) < 1e-10f);
	d.friction = .5f;
	d.tangentDamping = 1000;
	CHECK(YMPME_SkinContact_SphereRigid(&d, p, 1, &sphere, YMPME_Vec3(0, 0, 2), f, 2, &result) ==
		  PME_OK);
	CHECK(f[0].y > 0 && result.probeWrench.torque.z < 0);
	CHECK(YMPME_VecDot(YMPME_VecAdd(result.probeWrench.force, f[0]),
					   YMPME_VecAdd(result.probeWrench.force, f[0])) < 1e-10f);
	old = result;
	memcpy(saved, f, sizeof f);
	CHECK(YMPME_SkinContact_SphereRigid(&d, p, 1, &sphere, YMPME_Vec3(NAN, 0, 0), f, 2, &result) ==
		  PME_INVALID_ARGUMENT);
	CHECK(!memcmp(&result, &old, sizeof old) && !memcmp(f, saved, sizeof f));
	puts("skin contact: stiffness, damping, no adhesion, friction cap, reaction wrench, rollback "
		 "passed");
	return 0;
}
