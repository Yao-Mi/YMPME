#ifndef YMPME_ENCODER_DEVICE_H
#define YMPME_ENCODER_DEVICE_H
#include "YMPME_Encoder.h"
#ifdef __cplusplus
extern "C"
{
#endif
	typedef struct pme_encoder_device_
	{
		pme_encoder encoder;
		double zero;
		int direction;
		float noise;
		uint32_t seed;
	} pme_encoder_device;
	/* zero为机械零位rad，direction=+1/-1；noise为角度标准差rad的均匀噪声。 */
	pme_result YMPME_EncoderDevice_Init(pme_encoder_device *device, unsigned ticks, double zero,
										int direction, float noise, uint32_t seed);
	/* 校零后首样本速度无效，避免零位变化制造速度尖峰；调用方也需重置自身累积last。 */
	pme_result YMPME_EncoderDevice_Zero(pme_encoder_device *device, double mechanicalAngle);
	pme_result YMPME_EncoderDevice_Measure(pme_encoder_device *device, double mechanicalAngle,
										   const pme_sensor_stamp *stamp, pme_encoder_sample *out);
#ifdef __cplusplus
}
#endif
#endif
