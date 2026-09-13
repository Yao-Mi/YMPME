/* 闭合凸三角网格；几何、预计算工作区均由调用方持有。 */
#ifndef YMPME_CONVEXMESH_H
#define YMPME_CONVEXMESH_H
#include "../CONFIG/YMPME_PubType.h"
typedef struct pme_rotation_ { pme_vec3 axis[3]; } pme_rotation;
typedef struct pme_aabb_ { pme_vec3 min, max; } pme_aabb;
typedef struct pme_convex_mesh_
{
	const pme_vec3* vertices;
	const uint32_t* indices; /* 32位索引；使用16位索引时为NULL。 */
	size_t vertexCount, triangleCount;
	pme_vec3* faceAxes;
	pme_vec3* edgeAxes;
	size_t faceCount, edgeCount;
	pme_aabb bounds;
	const uint16_t* indices16; /* 16位索引；与indices恰有一个非NULL，数据由调用方持有。 */
} pme_convex_mesh;
/* Mesh must be initialized; offset is a triangle-list offset below triangleCount*3. */
static inline uint32_t YMPME_ConvexMesh_Index(const pme_convex_mesh* mesh,size_t offset)
{
    return mesh->indices16 ? mesh->indices16[offset] : mesh->indices[offset];
}
static inline int YMPME_ConvexMesh_HasIndices(const pme_convex_mesh* mesh)
{
    return mesh && ((mesh->indices != NULL) != (mesh->indices16 != NULL));
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
	const pme_vec3* vertices, size_t vertexCount, const uint32_t* indices, size_t triangleCount,
	pme_vec3* faceAxes, size_t faceCapacity, pme_vec3* edgeAxes, size_t edgeCapacity);
/* Same validation and lifetime as Init; borrows uint16 indices without copying/widening.
 * At most 65536 vertices (index 65535 is valid); each index must be < vertexCount.
 * Both widths may coexist in the same world and use the same compiled library. */
pme_result YMPME_ConvexMesh_Init16(pme_convex_mesh* mesh,
    const pme_vec3* vertices, size_t vertexCount, const uint16_t* indices, size_t triangleCount,
    pme_vec3* faceAxes, size_t faceCapacity, pme_vec3* edgeAxes, size_t edgeCapacity);

#ifdef __cplusplus
}
#endif
#endif
