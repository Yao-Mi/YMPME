#include "MODEL/SURFACE_ATLAS/YMPME_TopologyGrid.h"
#include <stdio.h>
#include <string.h>
#define N 256
#define CHECK(x)                                                                                   \
	do                                                                                             \
	{                                                                                              \
		if (!(x))                                                                                  \
		{                                                                                          \
			fprintf(stderr, "topology FAIL %d: %s\n", __LINE__, #x);                               \
			return 1;                                                                              \
		}                                                                                          \
	} while (0)
static int32_t adj[N * 3];
static uint32_t cells[512], faces[N], order[N], saved[512];
static float xy[N * 2], next[N * 2];
static pme_topology_grid_workspace work = {xy, next, order, N};
static int verify(size_t n, size_t count)
{
	unsigned seen[N] = {0};
	size_t populated = 0;
	for (size_t c = 0; c < count; c++)
	{
		uint32_t f = cells[c];
		if (f == PME_TOPOLOGY_EMPTY)
			continue;
		CHECK(f < n);
		CHECK(++seen[f] == 1);
		CHECK(faces[f] == c);
		populated++;
	}
	CHECK(populated == n);
	for (size_t f = 0; f < n; f++)
		CHECK(seen[f] == 1);
	return 0;
}
int main(void)
{
	pme_topology_grid_desc d = YMPME_TopologyGrid_DefaultDesc();
	d.width = 16;
	d.height = 16;
	pme_topology_grid_result r, copy;
	for (int i = 0; i < N; i++)
	{
		adj[i * 3] = i ? i - 1 : -1;
		adj[i * 3 + 1] = i < N - 1 ? i + 1 : -1;
		adj[i * 3 + 2] = -1;
	}
	CHECK(YMPME_TopologyGrid_Build(&d, adj, N, &work, cells, 512, faces, N, &r) == PME_OK);
	CHECK(verify(N, 256) == 0 && r.edges == 255 && r.meanDistance < 3);
	memcpy(saved, cells, 256 * sizeof(uint32_t));
	copy = r;
	CHECK(YMPME_TopologyGrid_Build(&d, adj, N, &work, cells, 512, faces, N, &r) == PME_OK);
	CHECK(!memcmp(saved, cells, 256 * sizeof(uint32_t)) && copy.mappingHash == r.mappingHash);
	CHECK(YMPME_TopologyGrid_Build(&d, adj, N, &work, cells, 255, faces, N, &r) ==
		  PME_CAPACITY_EXCEEDED);
	CHECK(!memcmp(saved, cells, 256 * sizeof(uint32_t)));
	adj[1] = N;
	CHECK(YMPME_TopologyGrid_Build(&d, adj, N, &work, cells, 512, faces, N, &r) ==
		  PME_INVALID_ARGUMENT);
	adj[1] = 1;
	adj[3] = -1;
	CHECK(YMPME_TopologyGrid_Build(&d, adj, N, &work, cells, 512, faces, N, &r) ==
		  PME_INVALID_ARGUMENT);
	adj[3] = 0;
	d.height = 24;
	CHECK(YMPME_TopologyGrid_Build(&d, adj, N, &work, cells, 512, faces, N, &r) == PME_OK);
	CHECK(verify(N, 384) == 0);
	/* Disconnected components and isolated faces retain one-to-one ownership. */
	for (int i = 0; i < N * 3; i++)
		adj[i] = -1;
	adj[0] = 1;
	adj[3] = 0;
	adj[6] = 3;
	adj[9] = 2;
	CHECK(YMPME_TopologyGrid_Build(&d, adj, 5, &work, cells, 512, faces, N, &r) == PME_OK);
	CHECK(verify(5, 384) == 0 && r.edges == 2);
	adj[0] = -1;
	d.width = d.height = 1;
	CHECK(YMPME_TopologyGrid_Build(&d, adj, 1, &work, cells, 512, faces, N, &r) == PME_OK);
	CHECK(verify(1, 1) == 0);
	puts("TOPOLOGY GRID PASS: unique ownership, complete coverage, stable inverse mapping, "
		 "deterministic, spare cells, disconnected graph, invalid inputs");
	return 0;
}
