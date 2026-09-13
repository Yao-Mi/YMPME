#ifndef YMPME_GRE_MESH_H
#define YMPME_GRE_MESH_H
#include "CORE/YMPME_ConvexMesh.h"
#include "YMGRE_OBJ.h"
#if YMPME_INDEX_BITS != YMGRE_INDEX_BITS
#error "Shared PME/GRE mesh requires matching index widths"
#endif
#ifdef __cplusplus
extern "C" {
#endif
/* Application-side adapter; GRE owns source vertices and all index arrays.
 * Initialize a fresh binding (or unbind first). Source is a single, local-space,
 * closed convex TRIANGLE mesh with shared vertex indices. No nextObject traversal.
 * Never call GRE LocalToWorld on the borrowed source; use PrepareRender instead.
 * All source buffers, source scale and topology are immutable until unbound.
 * Physics world must stop referencing mesh before Unbind/source destruction.
 * Separate axis caches and render-face scratch are caller-owned and must not alias source.
 * The render shell is BORROWED: never pass it to YMGRE_Free_Object.
 * PrepareRender is a flat-shading path preserving base UVs; advanced attributes are not rendered.
 */
typedef struct pme_gre_mesh_ {
    pme_convex_mesh mesh;
    GRE_Object4d source;
    gre_object4d render;
    gre_polygon4d* renderFaces;
    GRE_Vertex4d workVertices;
    float unitsPerMeter, sourceScale;
} pme_gre_mesh;
pme_result YMPME_GRE_Mesh_Bind(pme_gre_mesh* binding,GRE_Object4d source,float unitsPerMeter,
    pme_vec3* faceAxes,size_t faceCapacity,pme_vec3* edgeAxes,size_t edgeCapacity,
    gre_polygon4d* renderFaces,size_t renderFaceCapacity);
/* Uses GRE's existing pointList_ as world-space work storage. Local data never changes.
 * Updates returned render shell; call before rendering, without concurrent workspace writes. */
pme_result YMPME_GRE_Mesh_PrepareRender(pme_gre_mesh* binding,pme_vec3 position,pme_rotation rotation);
void YMPME_GRE_Mesh_Unbind(pme_gre_mesh* binding);
#ifdef __cplusplus
}
#endif
#endif
