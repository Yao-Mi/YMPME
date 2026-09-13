#include "SENSOR/TACTILE/YMPME_SurfaceTactile.h"
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
	pme_surface_patch p[2] = {{{1, 0, 0}, {0, 0, 1}, {0}, .01f}, {{0, 1, 0}, {0, 1, 0}, {0}, .02f}};
	pme_vec3 f[2] = {{2, 0, -3}, {0, -4, 0}};
	pme_surface_taxel cells[2], saved[2];
	pme_surface_tactile_frame out = {0}, old;
	out.cells = cells;
	out.capacity = 2;
	pme_sensor_stamp stamp = {1, 1.1, 1, 1};
	CHECK(YMPME_SurfaceTactile_Measure(p, f, 2, &stamp, (pme_vec3){0}, 1, &out) == PME_OK);
	CHECK(NEAR(cells[0].pressure, 300) && NEAR(cells[1].pressure, 200) && cells[0].shear.x == 2);
	CHECK(out.wrench.force.x == 2 && out.wrench.force.y == -4 && out.wrench.force.z == -3 &&
		  out.wrench.torque.y == 3 && out.approximate && out.stamp.availableTime == 1.1);
	old = out;
	memcpy(saved, cells, sizeof cells);
	p[1].area = 0;
	CHECK(YMPME_SurfaceTactile_Measure(p, f, 2, &stamp, (pme_vec3){0}, 0, &out) ==
		  PME_INVALID_ARGUMENT);
	CHECK(!memcmp(&out, &old, sizeof out) && !memcmp(saved, cells, sizeof cells));
	puts("surface tactile: curved normals, area pressure, shear, wrench, atomic error passed");
	return 0;
}
