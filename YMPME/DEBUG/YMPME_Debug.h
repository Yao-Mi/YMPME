#ifndef YMPME_DEBUG_H
#define YMPME_DEBUG_H
#include "../CORE/YMPME_World.h"
#include "YMPME_Log.h"
#ifdef __cplusplus
extern "C" {
#endif
/** 动态物体的平移动能总和（J），不包含势能。 */
double YMPME_Debug_KineticEnergy(const pme_world* world);
#ifdef __cplusplus
}
#endif
#endif
