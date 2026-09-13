#ifndef YMPME_SUSPENSION_H
#define YMPME_SUSPENSION_H
#include "../../CORE/YMPME_MathBase.h"
#ifdef __cplusplus
extern "C"
{
#endif
	typedef struct pme_suspension_desc_
	{
		float stiffness, damping, rest, travel, bumpMultiplier, maxForce;
	} pme_suspension_desc;
	/** 只推不拉的弹簧/阻尼/缓冲块载荷。k:N/m，c:N*s/m，rest/travel/distance:m，
	 * compressionRate:m/s（压缩为正）；maxForce:N。描述参数有限且非负，travel<=rest；distance/rate 有限，可为负。
	 * 成功写 force；非法参数返回 PME_INVALID_ARGUMENT 且不修改输出。零分配。 */
	pme_result YMPME_Suspension_Force(const pme_suspension_desc *desc, float distance, float compressionRate,
									  float *force);
#ifdef __cplusplus
}
#endif
#endif
