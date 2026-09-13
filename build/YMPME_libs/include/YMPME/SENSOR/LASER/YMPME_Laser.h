#ifndef YMPME_LASER_H
#define YMPME_LASER_H
#include "../YMPME_Sensor.h"
#include "../../CORE/YMPME_Ray.h"
#ifdef __cplusplus
extern "C"
{
#endif
	typedef struct pme_laser_desc_
	{
		pme_sensor_mount mount; /* Sensor local +Z is the laser direction. */
		float minRange, maxRange, bias, noise,
			resolution; /* metres; noise=stddev, resolution=0 disables quantization */
	} pme_laser_desc;
	typedef enum pme_laser_status_
	{
		PME_LASER_VALID,
		PME_LASER_NO_RETURN,
		PME_LASER_BLIND,
		PME_LASER_OUT_OF_RANGE
	} pme_laser_status;
	typedef struct pme_laser_sample_
	{
		pme_sensor_stamp stamp;
		float distance; /* measured metres, NAN when valid=0; never reuse a stale reading */
		int valid;
		pme_laser_status status;
		/* Geometric trace for optional debug rendering, before bias/noise/quantization.
		 * Blind-zone hits still stop the beam. No return: rayEnd is at maxRange. */
		pme_vec3 origin, direction, rayEnd;
		int hit;
		size_t body; /* provider ID when hit, otherwise SIZE_MAX */
	} pme_laser_sample;
	pme_laser_desc YMPME_Laser_DefaultDesc(void);
	/* One nearest-hit ray per sample, no scanning/heap/render/World dependency.
	 * Uses caller-supplied provider, simulation timestamp and independent RNG seed.
	 * Scalar distance is geometric range + bias + uniform noise, then quantized.
	 * Physical blind-zone obstruction is never skipped to find a farther target.
	 * NO_RETURN/BLIND/OUT_OF_RANGE are successful samples (stamp.valid=1, valid=0).
	 * Invalid arguments/provider failure: output and seed unchanged. Check return.
	 * Input/output buffers must not overlap. Caller handles sampling rate/latency
	 * through the shared sensor clock; no hidden wall clock or delayed queue. */
	pme_result YMPME_Laser_Measure(const pme_laser_desc *desc, const pme_feedback_state *state,
								   const pme_sensor_stamp *stamp, const pme_ray_provider *provider,
								   uint32_t *seed, pme_laser_sample *out);
#ifdef __cplusplus
}
#endif
#endif
