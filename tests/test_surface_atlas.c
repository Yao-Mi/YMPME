#include "MODEL/SURFACE_ATLAS/YMPME_SurfaceAtlas.h"
#include <stdio.h>
#include <string.h>
#define N 800
#define CHECK(x)                                                                                   \
	do                                                                                             \
	{                                                                                              \
		if (!(x))                                                                                  \
		{                                                                                          \
			fprintf(stderr, "atlas FAIL %d: %s\n", __LINE__, #x);                                  \
			return 1;                                                                              \
		}                                                                                          \
	} while (0)
static pme_vec3 vertices[1000];
static uint32_t indices[N * 3], ids[N], canonical[1000], queue[N];
static pme_surface_atlas_edge edges[N * 3];
static pme_surface_atlas_face faces[N];
static pme_surface_atlas_chart charts[N];
static float uv[N * 6], saved[N * 6];
static uint64_t occupied[256 * 4], shape[256 * 4];
static pme_surface_atlas_workspace work = {canonical, queue, edges, faces, charts, 1000, N};
static float area(size_t f)
{
	const float *p = uv + f * 6;
	return fabsf((p[2] - p[0]) * (p[5] - p[1]) - (p[4] - p[0]) * (p[3] - p[1])) * .5f;
}
static int verify(size_t n, const pme_surface_atlas_result *r)
{
	for (size_t f = 0; f < n; f++)
	{
		CHECK(fabsf(area(f) / r->triangleArea - 1) < .0001f);
		for (int k = 0; k < 6; k++)
			CHECK(uv[f * 6 + k] >= 0 && uv[f * 6 + k] <= 1);
		for (size_t g = 0; g < f; g++)
			if (ids[f] == ids[g])
			{
				int a = faces[f].xy[0][0] + faces[f].xy[1][0] + faces[f].xy[2][0],
					b = faces[g].xy[0][0] + faces[g].xy[1][0] + faces[g].xy[2][0];
				int c = faces[f].xy[0][1] + faces[f].xy[1][1] + faces[f].xy[2][1],
					d = faces[g].xy[0][1] + faces[g].xy[1][1] + faces[g].xy[2][1];
				CHECK(a != b || c != d);
			}
	}
	CHECK(r->occupancy > 0 && r->occupancy <= 1);
	return 0;
}
int main(void)
{
	pme_surface_atlas_desc d = YMPME_SurfaceAtlas_DefaultDesc();
	pme_surface_atlas_result r;
	/* Strongly unequal physical areas, duplicated UV seam vertices. */
	vertices[0] = YMPME_Vec3(0, 0, 0);
	vertices[1] = YMPME_Vec3(100, 0, 0);
	vertices[2] = YMPME_Vec3(0, 1, 0);
	vertices[3] = vertices[1];
	vertices[4] = YMPME_Vec3(100, .01f, 0);
	vertices[5] = vertices[2];
	for (int i = 0; i < 6; i++)
		indices[i] = (uint32_t)i;
	CHECK(YMPME_SurfaceAtlas_Build(&d, vertices, 6, indices, 2, NULL, &work, uv, ids, N, &r) ==
		  PME_OK);
	CHECK(r.charts == 1 && r.joinedEdges == 1);
	CHECK(verify(2, &r) == 0);
	/* Dense manifold grid; all triangles join on the unconstrained lattice. */
	for (int y = 0; y <= 20; y++)
		for (int x = 0; x <= 20; x++)
			vertices[y * 21 + x] = YMPME_Vec3(x, y, 0);
	size_t n = 0;
	for (int y = 0; y < 20; y++)
		for (int x = 0; x < 20; x++)
		{
			uint32_t a = (uint32_t)(y * 21 + x), b = a + 1, c = a + 21, e = c + 1;
			indices[n++] = a;
			indices[n++] = b;
			indices[n++] = c;
			indices[n++] = b;
			indices[n++] = e;
			indices[n++] = c;
		}
	d.maxFacesPerChart = 0;
	CHECK(YMPME_SurfaceAtlas_Build(&d, vertices, 441, indices, N, NULL, &work, uv, ids, N, &r) ==
		  PME_OK);
	CHECK(r.charts == 1 && r.joinedEdges == r.adjacentEdges);
	CHECK(verify(N, &r) == 0);
	d.maxFacesPerChart = 64;
	CHECK(YMPME_SurfaceAtlas_Build(&d, vertices, 441, indices, N, NULL, &work, uv, ids, N, &r) ==
		  PME_OK);
	CHECK(r.charts >= 13 && r.joinedEdges > r.adjacentEdges * .7f);
	CHECK(verify(N, &r) == 0);
	float beforePacking = r.occupancy;
	size_t oldJoined = r.joinedEdges, oldCharts = r.charts;
	CHECK(YMPME_SurfaceAtlas_Compact(&d, &work, N, 256, 4, occupied, shape, 256 * 4, uv, &r) ==
		  PME_OK);
	CHECK(r.occupancy >= beforePacking && r.joinedEdges == oldJoined && r.charts == oldCharts);
	CHECK(verify(N, &r) == 0);
	memcpy(saved, uv, sizeof uv);
	CHECK(YMPME_SurfaceAtlas_Build(&d, vertices, 441, indices, N, NULL, &work, uv, ids, N, &r) ==
		  PME_OK);
	CHECK(YMPME_SurfaceAtlas_Compact(&d, &work, N, 256, 4, occupied, shape, 256 * 4, uv, &r) ==
		  PME_OK);
	CHECK(memcmp(saved, uv, sizeof uv) == 0);
	CHECK(YMPME_SurfaceAtlas_Build(&d, vertices, 441, indices, N, NULL, &work, uv, ids, N - 1,
								   &r) == PME_CAPACITY_EXCEEDED);
	CHECK(memcmp(saved, uv, sizeof uv) == 0);
	indices[0] = 9999;
	CHECK(YMPME_SurfaceAtlas_Build(&d, vertices, 441, indices, N, NULL, &work, uv, ids, N, &r) ==
		  PME_INVALID_ARGUMENT);
	CHECK(memcmp(saved, uv, sizeof uv) == 0);

	vertices[0] = YMPME_Vec3(0, 0, 0);
	vertices[1] = YMPME_Vec3(1, 0, 0);
	vertices[2] = YMPME_Vec3(0, 1, 0);
	vertices[3] = YMPME_Vec3(0, 0, 1);
	const uint32_t tetra[] = {0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3};
	memcpy(indices, tetra, sizeof tetra);
	CHECK(YMPME_SurfaceAtlas_Build(&d, vertices, 4, indices, 4, NULL, &work, uv, ids, N, &r) ==
		  PME_OK);
	CHECK(r.adjacentEdges == 6 && r.joinedEdges >= 3);
	CHECK(verify(4, &r) == 0);
	uint32_t regions[] = {0, 1, 2, 3};
	CHECK(YMPME_SurfaceAtlas_Build(&d, vertices, 4, indices, 4, regions, &work, uv, ids, N, &r) ==
		  PME_OK);
	CHECK(r.charts == 4 && r.joinedEdges == 0);
	CHECK(verify(4, &r) == 0);
	vertices[4] = YMPME_Vec3(0, -1, 0);
	const uint32_t nonmanifold[] = {0, 1, 2, 1, 0, 3, 0, 1, 4};
	memcpy(indices, nonmanifold, sizeof nonmanifold);
	CHECK(YMPME_SurfaceAtlas_Build(&d, vertices, 5, indices, 3, NULL, &work, uv, ids, N, &r) ==
		  PME_OK);
	CHECK(r.nonManifoldEdges == 1 && r.charts == 3);
	CHECK(verify(3, &r) == 0);
	puts("SURFACE_ATLAS PASS: equal area, seam welding, adjacency, chart limit, no overlap, "
		 "deterministic, validation");
	return 0;
}
