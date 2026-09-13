#ifndef YMPME_SURFACE_TACTILE_H
#define YMPME_SURFACE_TACTILE_H
#include "../YMPME_Sensor.h"
#include "../../CORE/YMPME_Surface.h"
#ifdef __cplusplus
extern "C"
{
#endif
	typedef struct pme_surface_taxel_
	{
		pme_vec3 force, shear;
		float normalForce, pressure;
	} pme_surface_taxel;
	typedef struct pme_surface_tactile_frame_
	{
		pme_sensor_stamp stamp;
		pme_surface_taxel *cells;
		size_t capacity, count;
		pme_wrench wrench;
		pme_vec3 origin;
		int approximate;
	} pme_surface_tactile_frame;
	/* 曲面任意面积单元：force/world，pressure=max(0,-dot(F,n))/area，Pa。
	 * 输入已存在的接触力；此函数不生成力，不依赖MODEL/SKIN。
	 * 汇总载荷关于origin，approximate由提供方传入。失败不修改任何输出。
	 * 输入/输出缓冲不重叠；单元顺序由应用维护，可用真实流形替换近似模型。 */
	pme_result YMPME_SurfaceTactile_Measure(const pme_surface_patch *patches,
											const pme_vec3 *forces, size_t count,
											const pme_sensor_stamp *stamp, pme_vec3 origin,
											int approximate, pme_surface_tactile_frame *out);
#ifdef __cplusplus
}
#endif
#endif
