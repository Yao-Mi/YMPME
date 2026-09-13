#include "YMPME_Flight.h"
#include "../../CORE/YMPME_MathBase.h"
#if PME_ENABLE_FLIGHT && PME_FLIGHT_ROTORS
pme_result YMPME_Flight_RotorLoad(const pme_rotor_desc *r, float speed, pme_flight_load *out)
{
	float length, thrust, reaction;
	pme_vec3 axis;
	if (!r || !out || !isfinite(speed) || !YMPME_VecFinite(r->axis) || !YMPME_VecFinite(r->position) ||
	    !isfinite(r->thrustFactor) || r->thrustFactor < 0 || !isfinite(r->reactionFactor) ||
	    r->reactionFactor < 0)
		return PME_INVALID_ARGUMENT;
	length = hypotf(hypotf(r->axis.x, r->axis.y), r->axis.z);
	if (!isfinite(length) || length < 1e-12f)
		return PME_INVALID_ARGUMENT;
	axis = YMPME_VecScale(r->axis, 1 / length);
	thrust = (float)((double)r->thrustFactor * speed * speed);
	reaction = (float)(-(double)r->reactionFactor * speed * fabs((double)speed));
	if (!isfinite(thrust) || !isfinite(reaction))
		return PME_INVALID_ARGUMENT;
	return YMPME_Flight_LoadAtPoint(YMPME_VecScale(axis, thrust), r->position, YMPME_VecScale(axis, reaction),
	                                out);
}
#endif
