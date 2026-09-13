#include "YMPME_Flight.h"
#include "../../CORE/YMPME_MathBase.h"
#if PME_ENABLE_FLIGHT
pme_flight_load YMPME_Flight_ZeroLoad(void)
{
	pme_flight_load load = {{0, 0, 0}, {0, 0, 0}};
	return load;
}
pme_result YMPME_Flight_AddLoads(const pme_flight_load *a, const pme_flight_load *b, pme_flight_load *out)
{
	pme_flight_load sum;
	if (!a || !b || !out || !YMPME_VecFinite(a->force) || !YMPME_VecFinite(a->torque) ||
	    !YMPME_VecFinite(b->force) || !YMPME_VecFinite(b->torque))
		return PME_INVALID_ARGUMENT;
	sum.force = YMPME_VecAdd(a->force, b->force);
	sum.torque = YMPME_VecAdd(a->torque, b->torque);
	if (!YMPME_VecFinite(sum.force) || !YMPME_VecFinite(sum.torque))
		return PME_INVALID_ARGUMENT;
	*out = sum;
	return PME_OK;
}
pme_result YMPME_Flight_LoadAtPoint(pme_vec3 force, pme_vec3 position, pme_vec3 freeTorque,
                                    pme_flight_load *out)
{
	pme_flight_load load;
	if (!out || !YMPME_VecFinite(force) || !YMPME_VecFinite(position) || !YMPME_VecFinite(freeTorque))
		return PME_INVALID_ARGUMENT;
	load.force = force;
	load.torque = YMPME_VecAdd(YMPME_VecCross(position, force), freeTorque);
	if (!YMPME_VecFinite(load.torque))
		return PME_INVALID_ARGUMENT;
	*out = load;
	return PME_OK;
}
#endif
