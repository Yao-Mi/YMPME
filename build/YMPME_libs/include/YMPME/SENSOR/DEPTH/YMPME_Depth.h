#ifndef YMPME_DEPTH_H
#define YMPME_DEPTH_H
#include "../YMPME_Sensor.h"
#include "../../CORE/YMPME_Ray.h"
#ifdef __cplusplus
extern "C"
{
#endif
	typedef struct pme_depth_desc_
	{
		pme_sensor_mount mount;
		size_t width, height;
		float fx, fy, cx, cy; /* 像素单位；以左上角为(0,0)，采样像素中心(u+.5,v+.5)。 */
		float nearDepth, farDepth, bias, noise; /* 光轴Z深度m；噪声均匀，noise为标准差。 */
	} pme_depth_desc;
	typedef struct pme_depth_frame_
	{
		pme_sensor_stamp stamp;
		pme_vec3 origin;
		pme_rotation rotation;
		float *depth;
		size_t capacity, width, height; /* 行优先，光轴Z深度，NAN=无效。 */
	} pme_depth_frame;
	/* 预计算针孔相机单位方向缓存；+Z前，+X右，+Y上，图像行向下。
	 * Measure时缓存必须来自同一套宽高/内参，期间不允许修改。 */
	pme_result YMPME_Depth_BuildRays(const pme_depth_desc *desc, pme_vec3 *rays, size_t capacity);
	pme_result YMPME_Depth_Measure(const pme_depth_desc *desc, const pme_vec3 *rays,
								   size_t rayCount, const pme_feedback_state *state,
								   const pme_sensor_stamp *stamp, const pme_ray_provider *provider,
								   uint32_t *seed, pme_depth_frame *out);
	/* 无效/量化为0/超过65535输出0，常用unitsPerMeter=1000得到uint16毫米图。 */
	pme_result YMPME_Depth_EncodeU16(const pme_depth_frame *frame, float unitsPerMeter,
									 uint16_t *out, size_t capacity);
	/* 输出安装坐标点云，与深度像素一一对应，无效像素输出(NAN,NAN,NAN)。 */
	pme_result YMPME_Depth_Unproject(const pme_depth_frame *frame, const pme_vec3 *rays,
									 size_t rayCount, pme_vec3 *out, size_t capacity);
#ifdef __cplusplus
}
#endif
#endif
