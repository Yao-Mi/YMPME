#include "grasp.h"
#include <string.h>
int Grasp_Geometry(grasp_geometry *g, int shape, float mass)
{
	memset(g, 0, sizeof(*g));
	/* Axis-symmetric extruded polygon: exact COM at zero, principal axes XYZ.
	 * A box and an unequal-sided chamfered convex prism, both with flat bases. */
	static const float rectangle[4][2] = {{-.2f, -.2f}, {.2f, -.2f}, {.2f, .2f}, {-.2f, .2f}};
	static const float bevel[8][2] = {{-.13f, -.20f}, {.13f, -.20f}, {.23f, -.10f}, {.23f, .10f},
									  {.13f, .20f},	  {-.13f, .20f}, {-.23f, .10f}, {-.23f, -.10f}};
	if (shape < 1 || shape > 2)
		return 0;
	const float (*polygon)[2] = shape == 1 ? rectangle : bevel;
	int n = shape == 1 ? 4 : 8;
	for (int i = 0; i < n; ++i)
	{
		g->vertices[i] = YMPME_Vec3(polygon[i][0], -.2f, polygon[i][1]);
		g->vertices[n + i] = YMPME_Vec3(polygon[i][0], .2f, polygon[i][1]);
	}
	size_t k = 0;
	for (int i = 1; i < n - 1; ++i)
	{
		g->indices[k++] = 0;
		g->indices[k++] = i;
		g->indices[k++] = i + 1;
		g->indices[k++] = n;
		g->indices[k++] = n + i + 1;
		g->indices[k++] = n + i;
	}
	for (int i = 0; i < n; ++i)
	{
		int j = (i + 1) % n;
		g->indices[k++] = i;
		g->indices[k++] = n + i;
		g->indices[k++] = n + j;
		g->indices[k++] = i;
		g->indices[k++] = n + j;
		g->indices[k++] = j;
	}
	if (YMPME_ConvexMesh_Init(&g->mesh, g->vertices, (size_t)2 * n, g->indices, k / 3, g->faceAxes,
							  28, g->edgeAxes, 84) != PME_OK)
		return 0;
	/* Polygon area integrals, uniform volume density; not box-approximated inertia. */
	double area2 = 0, x2 = 0, z2 = 0;
	for (int i = 0; i < n; ++i)
	{
		int j = (i + 1) % n;
		double x = polygon[i][0], z = polygon[i][1], xx = polygon[j][0], zz = polygon[j][1],
			   d = x * zz - xx * z;
		area2 += d;
		x2 += d * (x * x + x * xx + xx * xx);
		z2 += d * (z * z + z * zz + zz * zz);
	}
	x2 /= 6 * area2;
	z2 /= 6 * area2;
	double y2 = .2 * .2 / 3;
	g->inertia =
		YMPME_Vec3((float)(mass * (y2 + z2)), (float)(mass * (x2 + z2)), (float)(mass * (x2 + y2)));
	return 1;
}
