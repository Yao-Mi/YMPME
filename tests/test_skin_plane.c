#include "CORE/YMPME_MathBase.h"
#include "MODEL/SKIN/YMPME_SkinPlane.h"
#include <stdio.h>
#include <string.h>
#define CHECK(x)                                                                                   \
	do                                                                                             \
	{                                                                                              \
		if (!(x))                                                                                  \
		{                                                                                          \
			fprintf(stderr, "plane FAIL %d: %s\n", __LINE__, #x);                                  \
			return 1;                                                                              \
		}                                                                                          \
	} while (0)
#define NEAR(a, b) (fabsf((a) - (b)) < 1e-5f)
int main(void)
{
	pme_skin_contact_desc d = {1000, 0, .5f, 100, 10000};
	pme_skin_plane plane = {{0, 1, 0}, {0, 0, 0}, {0, 0, 0}, 0, .01f};
	pme_surface_patch p[2] = {{{1, .005f, 0}, {0, -1, 0}, {0, 0, 0}, .1f},
							  {{0, .02f, 0}, {0, -1, 0}, {0, 0, 0}, .1f}};
	pme_vec3 f[2], old[2];
	pme_skin_contact_result result, saved;
	CHECK(YMPME_SkinContact_Plane(&d, p, 2, &plane, f, 2, &result) == PME_OK);
	CHECK(NEAR(f[0].y, .5f) && NEAR(f[1].y, 0) && result.contacts == 1);
	CHECK(NEAR(result.probeWrench.force.y, -.5f) && NEAR(result.probeWrench.torque.z, -.5f));
	p[0].velocity.x = 2;
	CHECK(YMPME_SkinContact_Plane(&d, p, 2, &plane, f, 2, &result) == PME_OK);
	CHECK(NEAR(f[0].x, -.25f)); /* Coulomb limit opposes slip. */
	plane.velocity.x = 3;
	CHECK(YMPME_SkinContact_Plane(&d, p, 2, &plane, f, 2, &result) == PME_OK && NEAR(f[0].x, .25f));
	d.maxPressure = 2;
	CHECK(YMPME_SkinContact_Plane(&d, p, 2, &plane, f, 2, &result) == PME_OK && result.saturated &&
		  NEAR(f[0].y, .2f));
	d.maxPressure = 10000;
	d.stiffness = 0;
	d.damping = 10;
	p[0].velocity.y = 1;
	CHECK(YMPME_SkinContact_Plane(&d, p, 2, &plane, f, 2, &result) == PME_OK && !result.contacts);
	p[0].velocity.y = -1;
	CHECK(YMPME_SkinContact_Plane(&d, p, 2, &plane, f, 2, &result) == PME_OK && NEAR(f[0].y, 1));
	p[0].normal.y = 1;
	CHECK(YMPME_SkinContact_Plane(&d, p, 2, &plane, f, 2, &result) == PME_OK && !result.contacts);
	memcpy(old, f, sizeof f);
	saved = result;
	p[1].area = -1;
	CHECK(YMPME_SkinContact_Plane(&d, p, 2, &plane, f, 2, &result) == PME_INVALID_ARGUMENT);
	CHECK(!memcmp(old, f, sizeof f) && !memcmp(&saved, &result, sizeof result));
	CHECK(YMPME_SkinContact_Plane(&d, p, 2, &plane, f, 1, &result) == PME_CAPACITY_EXCEEDED);
	p[1].area = .1f;
	plane.normal.y = 2;
	CHECK(YMPME_SkinContact_Plane(&d, p, 2, &plane, f, 2, &result) == PME_INVALID_ARGUMENT);
	puts("Plane contact: normal load, slip friction, damping, release, reaction torque, validation "
		 "PASS");
	return 0;
}
