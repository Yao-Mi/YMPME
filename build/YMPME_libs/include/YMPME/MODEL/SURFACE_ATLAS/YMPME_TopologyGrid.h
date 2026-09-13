#ifndef YMPME_TOPOLOGY_GRID_H
#define YMPME_TOPOLOGY_GRID_H
#include "../../CORE/YMPME_MathBase.h"
#ifdef __cplusplus
extern "C"
{
#endif
#define PME_TOPOLOGY_EMPTY UINT32_MAX
	/* Graph input: three reciprocal neighbours per face, -1 for a boundary.
	 * No positions or force values: topology alone determines the fixed mapping.
	 * Lazy graph diffusion with orthogonalization -> balanced grid assignment ->
	 * swaps minimizing squared grid distance of mesh neighbours. This is a heuristic
	 * graph embedding, not a guarantee that every grid neighbour is a mesh neighbour.
	 */
	typedef struct pme_topology_grid_desc_
	{
		uint32_t width, height, diffusionSteps, swapPasses, seed;
	} pme_topology_grid_desc;
	typedef struct pme_topology_grid_workspace_
	{
		float *xy, *next; /* each 2*faceCapacity */
		uint32_t *order;  /* faceCapacity */
		size_t faceCapacity;
	} pme_topology_grid_workspace;
	typedef struct pme_topology_grid_result_
	{
		size_t faces, cells, edges;
		float initialMeanDistance, meanDistance, maxDistance, withinTwoFraction;
		uint64_t mappingHash;
	} pme_topology_grid_result;
	pme_topology_grid_desc YMPME_TopologyGrid_DefaultDesc(void);
	/* Each face appears exactly once, each cell has at most one face; empty cells use
	 * PME_TOPOLOGY_EMPTY. Outputs: cellToFace[width*height], faceToCell[faceCount].
	 * Capacities are in elements. All buffers must be distinct/nonoverlapping.
	 * Validation precedes writes. Deterministic for the same build/input/seed/settings.
	 * Init only, no allocation or GRE/SKE dependency. */
	pme_result YMPME_TopologyGrid_Build(const pme_topology_grid_desc *desc,
										const int32_t *neighbours, size_t faceCount,
										pme_topology_grid_workspace *work, uint32_t *cellToFace,
										size_t cellCapacity, uint32_t *faceToCell,
										size_t faceCapacity, pme_topology_grid_result *out);
#ifdef __cplusplus
}
#endif
#endif
