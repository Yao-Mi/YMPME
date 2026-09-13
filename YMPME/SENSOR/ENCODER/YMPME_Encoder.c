#include "YMPME_Encoder.h"
#include <string.h>
#define TAU 6.28318530717958647692
pme_result YMPME_Encoder_Init(pme_encoder *s, unsigned ticks)
{
	if (!s || ticks > PME_ENCODER_COUNTS_PER_TURN)
		return PME_INVALID_ARGUMENT;
	memset(s, 0, sizeof(*s));
	s->ticksPerTurn = ticks ? ticks : PME_ENCODER_COUNTS_PER_TURN;
	return PME_OK;
}
pme_result YMPME_Encoder_Measure(pme_encoder *s, double angle, const pme_sensor_stamp *stamp,
								 pme_encoder_sample *out)
{
	if (!s || !stamp || !out || !s->ticksPerTurn || s->ticksPerTurn > PME_ENCODER_COUNTS_PER_TURN ||
		!stamp->valid || !isfinite(angle) || !isfinite(stamp->time) || stamp->time < 0 ||
		!isfinite(stamp->availableTime) || stamp->availableTime < stamp->time ||
		(s->initialized && stamp->time <= s->lastTime))
		return PME_INVALID_ARGUMENT;
	double wrapped = fmod(angle, TAU);
	if (wrapped < 0)
		wrapped += TAU;
	double tick = floor(wrapped / TAU * s->ticksPerTurn + .5);
	if (tick >= s->ticksPerTurn)
		tick = 0;
	uint32_t code = (uint32_t)floor(tick / s->ticksPerTurn * PME_ENCODER_COUNTS_PER_TURN + .5);
	pme_encoder_sample sample = {0};
	sample.stamp = *stamp;
	sample.angle = (uint16_t)(code % PME_ENCODER_COUNTS_PER_TURN);
	uint16_t now = sample.angle, last = s->lastAngle;
	/* 当前GCC补码目标：转为int16_t得到带符号的单圈最短增量。 */
	int16_t delta = (int16_t)(now - last);
	sample.velocity = s->initialized
		? (float)(delta * (TAU / PME_ENCODER_COUNTS_PER_TURN) / (stamp->time - s->lastTime))
		: 0;
	sample.velocityValid = s->initialized;
	if (!isfinite(sample.velocity))
		return PME_INVALID_ARGUMENT;
	s->initialized = 1;
	s->lastAngle = now;
	s->lastTime = stamp->time;
	*out = sample;
	return PME_OK;
}
