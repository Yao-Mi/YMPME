/* 固定65x65高度场，间距0.375m、起点(-12,-12)m；height连续数组由实例持有。
 * InitHills 是原测试场的山包预设；亦可自行填充有限高度值，{0}即平面。
 * Sample采用两三角形插值，height必填、normal可空；域外/非法坐标返回0且不改输出。
 * 不包含GRE网格或文件I/O；PGM读写仍属于应用层。 */
#ifndef YMPME_TERRAIN_H
#define YMPME_TERRAIN_H
#include "../../CORE/YMPME_MathBase.h"
#define PME_TERRAIN_N 65
#define PME_TERRAIN_STEP 0.375f
#define PME_TERRAIN_MIN (-12.0f)
#ifdef __cplusplus
extern "C"
{
#endif
	typedef struct pme_terrain_
	{
		float height[PME_TERRAIN_N * PME_TERRAIN_N];
	} pme_terrain;
	void YMPME_Terrain_InitHills(pme_terrain *t);
	/** 与渲染网格相同的三角形重心插值；域外返回0，不修改输出。 */
	int YMPME_Terrain_Sample(const pme_terrain *t, float x, float z, float *height, pme_vec3 *normal);
#ifdef __cplusplus
}
#endif
#endif
