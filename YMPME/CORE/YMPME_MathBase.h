/* 无分配的三维向量运算。 */
#ifndef YMPME_MATHBASE_H
#define YMPME_MATHBASE_H
#include "../CONFIG/YMPME_PubType.h"
#include <math.h>
static inline pme_vec3 YMPME_Vec3(float x, float y, float z)
{
	pme_vec3 v = {x, y, z};
	return v;
}
static inline pme_vec3 YMPME_VecAdd(pme_vec3 a, pme_vec3 b)
{
	return YMPME_Vec3(a.x+b.x, a.y+b.y, a.z+b.z);
}
static inline pme_vec3 YMPME_VecSub(pme_vec3 a, pme_vec3 b)
{
	return YMPME_Vec3(a.x-b.x, a.y-b.y, a.z-b.z);
}
static inline pme_vec3 YMPME_VecScale(pme_vec3 a, float s)
{
	return YMPME_Vec3(a.x*s, a.y*s, a.z*s);
}
static inline float YMPME_VecDot(pme_vec3 a, pme_vec3 b)
{
	return a.x*b.x + a.y*b.y + a.z*b.z;
}
static inline pme_vec3 YMPME_VecCross(pme_vec3 a, pme_vec3 b)
{
	return YMPME_Vec3(a.y*b.z-a.z*b.y, a.z*b.x-a.x*b.z, a.x*b.y-a.y*b.x);
}
static inline int YMPME_VecFinite(pme_vec3 a)
{
	return isfinite(a.x) && isfinite(a.y) && isfinite(a.z);
}
static inline float YMPME_Clamp(float x, float low, float high)
{
	return fmaxf(low, fminf(x, high));
}
#endif
