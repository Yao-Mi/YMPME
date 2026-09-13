/* 射线提供者协议；无World/GRE依赖。输入/输出均为世界坐标、米。 */
#ifndef YMPME_RAY_H
#define YMPME_RAY_H
#include "YMPME_MathBase.h"
typedef struct pme_ray_
{
	pme_vec3 origin, direction;
	float maxDistance;
} pme_ray;
typedef struct pme_ray_hit_
{
	int hit;
	float distance;
	pme_vec3 point, normal;
	size_t body, primitive; /* 提供者定义ID；未命中/Sphere/Box的primitive=SIZE_MAX。 */
} pme_ray_hit;
/* direction必须为单位向量；返回最近的非负距离表面，内部起点返回出口。
 * 不得修改场景；PME_OK且hit=0表示正常未命中。错误不可伪装为未命中。
 * 几何采样应在同一个快照/物理步末完成，调用方负责避免并发修改。 */
typedef pme_result (*pme_ray_cast_fn)(void *context, const pme_ray *ray, pme_ray_hit *out);
typedef struct pme_ray_provider_
{
	pme_ray_cast_fn cast;
	void *context;
} pme_ray_provider;
#endif
