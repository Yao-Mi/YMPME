#ifndef PME_DEMO_FORCE_CHANNELS_H
#define PME_DEMO_FORCE_CHANNELS_H
#include "CORE/YMPME_MathBase.h"
#include "YMGRE_PubType.h"
/* 世界力转UV正交切向(U右,V下)，z为法向压缩；保留镜像UV方向。 */
int DemoForce_Local(const pme_vec3 p[3], const float uv[6], pme_vec3 normal, pme_vec3 force,
					pme_vec3 *out);
/* R:[0,normalRange]，G/B:[-tangentRange,+tangentRange]；零切向=128。 */
GRErgb24 DemoForce_RGB(pme_vec3 local, float normalRange, float tangentRange, int *clipped);
#endif
