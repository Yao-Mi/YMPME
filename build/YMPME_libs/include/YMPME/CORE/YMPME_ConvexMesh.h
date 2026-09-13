/* 闭合凸三角网格；几何、预计算工作区均由调用方持有。 */
#ifndef YMPME_CONVEXMESH_H
#define YMPME_CONVEXMESH_H
#include "../CONFIG/YMPME_PubType.h"
#include "../CONFIG/YMPME_Index.h"
#include <string.h>
typedef struct pme_rotation_ { pme_vec3 axis[3]; } pme_rotation;
typedef struct pme_aabb_ { pme_vec3 min, max; } pme_aabb;
/* Borrowed local-space geometry. No ownership or renderer dependency.
 * Positions are three consecutive floats at positionOffset in each vertex record.
 * Supply either flat triangle indices, or strided face records containing an index pointer.
 * Each face must already be a triangle. All buffers remain immutable while in use. */
typedef struct pme_mesh_view_ {
    const void* vertices;
    size_t vertexCount, vertexStride, positionOffset;
    float vertexScale;
    const PME_Index* indices;
    const void* triangles;
    size_t triangleCount, triangleStride, indexPointerOffset;
} pme_mesh_view;
typedef struct pme_convex_mesh_ {
    const pme_vec3* vertices;
    const PME_Index* indices;
    size_t vertexCount, triangleCount;
    pme_vec3* faceAxes;
    pme_vec3* edgeAxes;
    size_t faceCount, edgeCount;
    pme_aabb bounds;
    pme_mesh_view view; /* Nonzero only for borrowed strided geometry. */
} pme_convex_mesh;
/* Initialized mesh and in-range offsets required; no allocation, no index conversion. */
static inline PME_Index YMPME_ConvexMesh_Index(const pme_convex_mesh* mesh,size_t offset)
{
    if(mesh->indices) return mesh->indices[offset];
    if(mesh->view.indices) return mesh->view.indices[offset];
    const PME_Index* row;
    memcpy(&row,(const unsigned char*)mesh->view.triangles +
        (offset/3)*mesh->view.triangleStride + mesh->view.indexPointerOffset,sizeof(row));
    return row[offset%3];
}
static inline pme_vec3 YMPME_ConvexMesh_Vertex(const pme_convex_mesh* mesh,size_t index)
{
    if(mesh->vertices) return mesh->vertices[index];
    pme_vec3 v;
    memcpy(&v,(const unsigned char*)mesh->view.vertices +
        index*mesh->view.vertexStride + mesh->view.positionOffset,sizeof(v));
    v.x*=mesh->view.vertexScale; v.y*=mesh->view.vertexScale; v.z*=mesh->view.vertexScale;
    return v;
}
static inline int YMPME_ConvexMesh_HasIndices(const pme_convex_mesh* mesh)
{
    return mesh && (mesh->indices || mesh->view.indices || mesh->view.triangles);
}
static inline int YMPME_ConvexMesh_HasVertices(const pme_convex_mesh* mesh)
{
    return mesh && (mesh->vertices || mesh->view.vertices);
}
#ifdef __cplusplus
extern "C" {
#endif
/* Closest geometric point on a finite, nondegenerate triangle; no allocation. */
pme_vec3 YMPME_Triangle_ClosestPoint(pme_vec3 p,pme_vec3 a,pme_vec3 b,pme_vec3 c);
pme_rotation YMPME_Rotation_Identity(void);
int YMPME_Rotation_IsValid(pme_rotation rotation);
pme_vec3 YMPME_Rotation_Apply(pme_rotation rotation, pme_vec3 point);
/** 校验有限坐标、非退化三角形、闭合拓扑及凸性；拒绝凹/开口/平面网格。
 * 顶点单位米，无隐含缩放；索引共享边必须共享相同顶点索引。
 * faceAxes/edgeAxes 为外部预计算缓存，建议容量 triangleCount 和 triangleCount*3。
 * 成功后全部缓存必须保持有效且不可修改，直到所有引用该 Mesh 的 Body 被移除。
 * 不申请内存；失败不修改 mesh，但可能写入工作缓存。
 */
pme_result YMPME_ConvexMesh_Init(pme_convex_mesh* mesh,
	const pme_vec3* vertices, size_t vertexCount, const PME_Index* indices, size_t triangleCount,
	pme_vec3* faceAxes, size_t faceCapacity, pme_vec3* edgeAxes, size_t edgeCapacity);
/* Same convexity/lifetime rules as Init; does not copy geometry.
 * Coordinates are scaled to meters when read. Invalid layout leaves mesh unchanged;
 * failed geometry validation may overwrite caller-provided axis scratch buffers. */
pme_result YMPME_ConvexMesh_InitView(pme_convex_mesh* mesh,const pme_mesh_view* view,
    pme_vec3* faceAxes,size_t faceCapacity,pme_vec3* edgeAxes,size_t edgeCapacity);
#ifdef __cplusplus
}
#endif
#endif
