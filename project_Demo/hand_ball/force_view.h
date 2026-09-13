#ifndef HAND_BALL_FORCE_VIEW_H
#define HAND_BALL_FORCE_VIEW_H
#include "CORE/YMPME_MathBase.h"
#include "YMGRE_PubType.h"
/* UV的+U右、+V下；局部z为压缩力。保留镜像岛的UV方向。 */
int HandForce_Local(const pme_vec3 triangle[3], const float uv[6], pme_vec3 normal, pme_vec3 force,
					pme_vec3 *out);
/* R=Fn [0,.10]N; G=Fu, B=Fv [-.05,+.05]N; G/B=128为零。 */
GRErgb24 HandForce_RGB(pme_vec3 local, int *clipped);
#endif
