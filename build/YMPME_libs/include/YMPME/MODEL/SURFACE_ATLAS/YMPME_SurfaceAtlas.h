#ifndef YMPME_SURFACE_ATLAS_H
#define YMPME_SURFACE_ATLAS_H
#include "../../CORE/YMPME_MathBase.h"
#ifdef __cplusplus
extern "C"
{
#endif
	/* Equal-area sensor display atlas. Metric shape/area is deliberately NOT preserved.
	 * Each face becomes a unit equilateral triangle. Neighbours unfold across shared edges,
	 * cutting seams when the triangular lattice is occupied or the chart size limit is reached. */
	typedef struct pme_surface_atlas_desc_
	{
		float weldTolerance, padding; /* SI positional welding tolerance; padding in unit edges */
		uint32_t maxFacesPerChart;	  /* 0 = no limit */
	} pme_surface_atlas_desc;
	typedef struct pme_surface_atlas_edge_
	{
		uint32_t a, b, face, slot;
	} pme_surface_atlas_edge;
	typedef struct pme_surface_atlas_face_
	{
		int32_t xy[3][2], neighbour[3], chart;
	} pme_surface_atlas_face;
	typedef struct pme_surface_atlas_chart_
	{
		float lowX, lowY, highX, highY, x, y;
		uint32_t count, packed;
	} pme_surface_atlas_chart;
	typedef struct pme_surface_atlas_workspace_
	{
		uint32_t *canonical, *queue;
		pme_surface_atlas_edge *edges; /* 3 * faceCapacity */
		pme_surface_atlas_face *faces;
		pme_surface_atlas_chart *charts;
		size_t vertexCapacity, faceCapacity;
	} pme_surface_atlas_workspace;
	typedef struct pme_surface_atlas_result_
	{
		size_t charts, adjacentEdges, joinedEdges, nonManifoldEdges;
		float triangleArea, occupancy; /* common UV area and sum of face UV areas */
	} pme_surface_atlas_result;
	pme_surface_atlas_desc YMPME_SurfaceAtlas_DefaultDesc(void);
	/* indices: faceCount*3; corners: faceCount*6 normalized UVs; chartIds: faceCount.
	 * regions optional: unequal regions force a seam. Duplicate seam vertices can be welded by
	 * position; tolerance 0 welds exactly coincident positions. Nonmanifold edges are cut.
	 * Caller owns all storage; inputs/outputs/workspace must not overlap. Output capacity is in
	 * faces; arrays in workspace as above. No heap allocation or GRE/SKE/SENSOR dependencies.
	 * Input validation precedes output writes; workspace may be changed on failure.
	 * Initialization cost O(V^2 + F log F + F*maxFacesPerChart + charts^2), not a per-frame task.
	 * This greedy unfold favours adjacency but does not minimize seams globally. */
	pme_result YMPME_SurfaceAtlas_Build(const pme_surface_atlas_desc *desc,
										const pme_vec3 *vertices, size_t vertexCount,
										const uint32_t *indices, size_t faceCount,
										const uint32_t *regions,
										pme_surface_atlas_workspace *workspace, float *corners,
										uint32_t *chartIds, size_t capacity,
										pme_surface_atlas_result *out);
	/* Optional outline-aware packing after Build, with the same unmodified workspace.
	 * Caller provides two bitmaps of side * ceil(side/64) uint64 words each.
	 * side <= 4096, pixelsPerEdge in [2,32]. Runs at initialization, no heap use.
	 * Packs conservative padded chart silhouettes into holes, preserving every seam and
	 * face orientation. Tries several widths and only commits a denser layout.
	 * UV/out must be the preceding Build outputs. The workspace may change.
	 */
	pme_result YMPME_SurfaceAtlas_Compact(const pme_surface_atlas_desc *desc,
										  pme_surface_atlas_workspace *workspace, size_t faceCount,
										  uint32_t side, uint32_t pixelsPerEdge, uint64_t *occupied,
										  uint64_t *shape, size_t wordCapacity, float *corners,
										  pme_surface_atlas_result *out);
#ifdef __cplusplus
}
#endif
#endif
