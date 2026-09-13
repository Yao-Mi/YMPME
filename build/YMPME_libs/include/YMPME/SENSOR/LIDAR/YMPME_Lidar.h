#ifndef YMPME_LIDAR_H
#define YMPME_LIDAR_H
#include "../YMPME_Sensor.h"
#include "../../CORE/YMPME_Ray.h"
#ifdef __cplusplus
extern "C"
{
#endif
	typedef struct pme_lidar_desc_
	{
		pme_sensor_mount mount;
		float minRange, maxRange, bias, noise;
	} pme_lidar_desc;
	typedef struct pme_lidar_point_
	{
		pme_vec3 point; /* 安装坐标；失效为零。帧origin/rotation可转世界坐标。 */
		float range;	/* m；失效为NAN。 */
		size_t body;
		int valid;
	} pme_lidar_point;
	typedef struct pme_lidar_frame_
	{
		pme_sensor_stamp stamp;
		pme_vec3 origin;
		pme_rotation rotation;
		pme_lidar_point *points;
		size_t capacity, count;
	} pme_lidar_frame;
	/* 2D水平面(XZ)360度单圈，0度朝+Z；count束，角步长2*pi/count，首尾不重复。 */
	pme_result YMPME_Lidar_Scan360(pme_vec3 *directions, size_t capacity, size_t count);
	/* 按 elevation行、azimuth列产生单位方向：+Z前、+X右、+Y上，角度rad。
	 * 全周使用azimuthStep=2*pi/columns，不重复首尾。方向缓存由调用方持有。
	 * pattern仅初始化时调用；Measure不重新计算三角函数。 */
	pme_result YMPME_Lidar_Pattern(pme_vec3 *directions, size_t capacity, size_t columns,
								   size_t rows, float azimuthStart, float azimuthStep,
								   float elevationStart, float elevationStep);
	/* 瞬时单回波扫描，保留每条射线下标，未命中/盲区有效位为0；近处遮挡不会透射。
	 * bias/noise单位m，noise为标准差指定的均匀噪声；seed独立且成功才提交。
	 * 每帧固定同一姿态，不模拟扫描运动畸变、反射强度或多回波。
	 * 外部缓冲不得互相重叠；容量/参数检查失败保持输出，提供者中途错误会使
	 * stamp.valid=0,count=0，缓存可能已部分写入。调用者必须检查返回值。 */
	pme_result YMPME_Lidar_Measure(const pme_lidar_desc *desc, const pme_vec3 *directions,
								   size_t count, const pme_feedback_state *state,
								   const pme_sensor_stamp *stamp, const pme_ray_provider *provider,
								   uint32_t *seed, pme_lidar_frame *out);
#ifdef __cplusplus
}
#endif
#endif
