#include "YMPME_Suspension.h"
pme_result YMPME_Suspension_Force(const pme_suspension_desc *d, float distance, float rate, float *out)
{
	if (!d || !out || !isfinite(distance) || !isfinite(rate) || !isfinite(d->stiffness) || d->stiffness < 0 ||
		!isfinite(d->damping) || d->damping < 0 || !isfinite(d->rest) || d->rest < 0 ||
		!isfinite(d->travel) || d->travel < 0 || d->travel > d->rest || !isfinite(d->bumpMultiplier) ||
		d->bumpMultiplier < 0 || !isfinite(d->maxForce) || d->maxForce < 0)
		return PME_INVALID_ARGUMENT;
	float force = 0;
	if (distance <= d->rest)
	{
		force = d->stiffness * YMPME_Clamp(d->rest - distance, 0, d->travel) + d->damping * rate;
		if (distance < d->rest - d->travel)
			force += (d->rest - d->travel - distance) * d->stiffness * d->bumpMultiplier;
		if (!isfinite(force))
			return PME_INVALID_ARGUMENT;
		force = YMPME_Clamp(force, 0, d->maxForce);
	}
	*out = force;
	return PME_OK;
}
