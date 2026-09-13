#include "YMPME_EncoderDevice.h"
#define TAU 6.2831853071795864769
pme_result YMPME_EncoderDevice_Init(pme_encoder_device *d, unsigned ticks, double zero,
									int direction, float noise, uint32_t seed)
{
	pme_encoder_device v = {0};
	if (!d || !isfinite(zero) || (direction != 1 && direction != -1) || !isfinite(noise) ||
		noise < 0 || YMPME_Encoder_Init(&v.encoder, ticks) != PME_OK)
		return PME_INVALID_ARGUMENT;
	v.zero = fmod(zero, TAU);
	v.direction = direction;
	v.noise = noise;
	v.seed = seed;
	*d = v;
	return PME_OK;
}
pme_result YMPME_EncoderDevice_Zero(pme_encoder_device *d, double angle)
{
	pme_encoder_device v;
	if (!d || !isfinite(angle))
		return PME_INVALID_ARGUMENT;
	v = *d;
	if (YMPME_Encoder_Init(&v.encoder, d->encoder.ticksPerTurn) != PME_OK)
		return PME_INVALID_ARGUMENT;
	v.zero = fmod(angle, TAU);
	*d = v;
	return PME_OK;
}
pme_result YMPME_EncoderDevice_Measure(pme_encoder_device *d, double angle,
									   const pme_sensor_stamp *stamp, pme_encoder_sample *out)
{
	pme_encoder_device next;
	pme_encoder_sample sample;
	double measured;
	if (!d || !out || !isfinite(angle) || !isfinite(d->zero) ||
		(d->direction != 1 && d->direction != -1) || !isfinite(d->noise) || d->noise < 0)
		return PME_INVALID_ARGUMENT;
	next = *d;
	measured = next.direction * (fmod(angle, TAU) - fmod(next.zero, TAU));
	if (next.noise > 0)
		measured += next.noise * YMPME_Sensor_UnitNoise(&next.seed);
	if (YMPME_Encoder_Measure(&next.encoder, measured, stamp, &sample) != PME_OK)
		return PME_INVALID_ARGUMENT;
	*d = next;
	*out = sample;
	return PME_OK;
}
