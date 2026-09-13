#ifndef YMPME_ENCODER_H
#define YMPME_ENCODER_H
#include "../YMPME_Sensor.h"
#define PME_ENCODER_COUNTS_PER_TURN UINT32_C(65536)
#ifdef __cplusplus
extern "C"
{
#endif
	typedef struct pme_encoder_sample_
	{
		pme_sensor_stamp stamp;
		uint16_t angle; /* 单圈码值0..65535，angle*360/65536得到度；360度回绕为0。 */
		float velocity; /* rad/s，按回绕码值差分；仅velocityValid非0时有效。 */
		int velocityValid;
	} pme_encoder_sample;
	typedef struct pme_encoder_
	{
		uint16_t lastAngle; /* 上次单圈码值，用int16_t差值处理回绕。 */
		double lastTime;
		unsigned ticksPerTurn;
		int initialized;
	} pme_encoder;
	/* 0或65536：默认16位精度；1..65536可模拟较低刻度数，输出仍缩放到16位单圈。
	 * Measure输入rad，可含多圈或负角度；最近刻度舍入，端点回绕。
	 * 首样本角度/时间戳有效，velocity=0且velocityValid=0；后续按实际间隔差分。
	 * 相邻真实转角须严格小于pi，单圈编码器不能辨别采样之间的整圈运动。 */
	pme_result YMPME_Encoder_Init(pme_encoder *sensor, unsigned ticksPerTurn);
	pme_result YMPME_Encoder_Measure(pme_encoder *sensor, double angle,
									 const pme_sensor_stamp *stamp, pme_encoder_sample *out);
#ifdef __cplusplus
}
#endif
#endif
