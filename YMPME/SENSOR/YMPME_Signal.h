/* 可选传感器的标量测量链；校准->一阶低通->限幅->量化，不改变物理真值。 */
#ifndef YMPME_SIGNAL_H
#define YMPME_SIGNAL_H
#include "YMPME_Sensor.h"
#ifdef __cplusplus
extern "C"
{
#endif
	typedef struct pme_signal_desc_
	{
		float gain, offset, minimum, maximum, resolution, cutoffHz;
	} pme_signal_desc;
	typedef struct pme_signal_state_
	{
		double time;
		float filtered;
		int initialized;
	} pme_signal_state;
	pme_signal_desc YMPME_Signal_DefaultDesc(void);
	int YMPME_Signal_DescValid(const pme_signal_desc *desc);
	/* resolution=0不量化，cutoffHz=0不过滤；截止频率单位Hz。
	 * 首样本直接初始化滤波值，之后使用实际时间差。clipped表示滤波后超量程。
	 * 所有配置/状态/输出缓冲不重叠；失败不修改状态和输出。 */
	pme_result YMPME_Signal_Process(const pme_signal_desc *desc, pme_signal_state *state,
									float input, double time, float *out, int *clipped);
#ifdef __cplusplus
}
#endif
#endif
