#ifndef YMPME_QUERY_H
#define YMPME_QUERY_H
#include "../YMPME_Ray.h"
#include "../YMPME_World.h"
#ifdef __cplusplus
extern "C"
{
#endif
	typedef struct pme_query_filter_
	{
		uint32_t category, mask, collisionGroup, ignoreAssembly;
		size_t ignoreBody;
		int includeSensors;
	} pme_query_filter;
	typedef struct pme_world_query_
	{
		const pme_world *world;
		pme_query_filter filter;
	} pme_world_query;
	pme_query_filter YMPME_Query_DefaultFilter(void);
	/* 球/轴对齐盒/旋转凸Mesh：AABB筛选后求精确表面交点。
	 * 光学查询始终精确，不受World.meshPrecision影响；遵守mask/group/assembly。
	 * 逐个遍历物体，无BVH、无分配，不修改World碰撞统计。 */
	pme_result YMPME_Query_WorldRaycast(void *context, const pme_ray *ray, pme_ray_hit *out);
#ifdef __cplusplus
}
#endif
#endif
