/* 世界坐标、SI；曲面采样协议，不依赖骨骼、渲染、接触模型或传感器。 */
#ifndef YMPME_SURFACE_H
#define YMPME_SURFACE_H
#include "YMPME_Feedback.h"
typedef struct pme_surface_patch_
{
	pme_vec3 position, normal, velocity;
	float area;
} pme_surface_patch;
#endif
