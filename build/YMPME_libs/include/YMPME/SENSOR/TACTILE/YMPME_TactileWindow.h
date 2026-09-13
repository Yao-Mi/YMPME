#ifndef YMPME_TACTILE_WINDOW_H
#define YMPME_TACTILE_WINDOW_H
#include "YMPME_Tactile.h"
#ifdef __cplusplus
extern "C"
{
#endif
	typedef struct pme_tactile_window_
	{
		pme_taxel *integrals;
		size_t capacity, width, height, accepted, rejected;
		double startTime, endTime, duration, availableTime;
		pme_vec3 origin;
		pme_rotation rotation;
		int active, approximate;
	} pme_tactile_window;
	/* 使用外部逐单元积分缓存；调用Begin开始新的窗口，不分配内存。 */
	pme_result YMPME_TactileWindow_Begin(pme_tactile_window *window, pme_taxel *integrals,
										 size_t capacity);
	/* 每个物理子步添加一次，按interval积分力和压力；漏步、重复步、容量不足、
	 * 非法帧/数值溢出均拒绝且不修改窗口。帧和积分缓存不得重叠。
	 * 允许皮肤随体移动：累计安装坐标分量，网格尺寸/安装配置在窗口内须保持不变。 */
	pme_result YMPME_TactileWindow_Add(pme_tactile_window *window, const pme_tactile_frame *frame,
									   float interval);
	/* 输出局部随体平均力/压力；origin/rotation为窗口末姿态，availability不早于
	 * 全部输入帧。Read不清空积分，输出cells不能与积分缓存重叠。 */
	pme_result YMPME_TactileWindow_Read(const pme_tactile_window *window, pme_tactile_frame *out);
#ifdef __cplusplus
}
#endif
#endif
