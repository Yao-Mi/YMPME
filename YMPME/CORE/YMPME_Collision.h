#ifndef YMPME_COLLISION_H
#define YMPME_COLLISION_H
#include "../OPOBJ/YMPME_OBJ.h"
typedef struct pme_collision_stats_
{
	uint64_t filteredPairs, aabbTests, aabbRejected, narrowphaseTests, meshTests;
} pme_collision_stats;
typedef struct pme_collision_options_
{
	int meshPrecision; /* 1：凸 Mesh SAT 精检测；0：涉及 Mesh 的配对只用 AABB 响应。 */
	const uint32_t* groupMasks; /* 借用 32 行配对掩码；NULL 允许所有组配对。 */
	pme_collision_stats* stats; /* 可选累计计数，查询无内存分配。 */
} pme_collision_options;
#ifdef __cplusplus
extern "C" {
#endif
/** 层过滤 + 同装配体过滤 + 可选世界组矩阵，均先于 AABB。 */
int YMPME_ShouldCollide(const pme_body* a,const pme_body* b,const pme_collision_options* options);
/** 获取世界 AABB。Mesh 使用旋转后的局部包围盒，保证保守覆盖。 */
int YMPME_Body_GetAABB(const pme_body* body,pme_aabb* bounds);
/** 默认启用 Mesh 精检测；无世界矩阵，仍遵守 category/mask 和 ignoreAssembly。 */
int YMPME_Collide(const pme_body* a,const pme_body* b,pme_contact* contact);
/** 所有形状经过 AABB 粗检测，只有候选配对进入精检测；contact 可 NULL。
 * 不相交时不修改 contact；闭合凸 Mesh 支持 Mesh/Box/Sphere，凹 Mesh 应拆成凸零件。
 */
int YMPME_CollideWithOptions(const pme_body* a,const pme_body* b,
	const pme_collision_options* options,pme_contact* contact);
#ifdef __cplusplus
}
#endif
#endif
