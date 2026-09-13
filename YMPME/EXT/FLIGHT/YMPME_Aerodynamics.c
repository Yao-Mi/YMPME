#include "YMPME_Flight.h"
#include "../../CORE/YMPME_MathBase.h"
#if PME_ENABLE_FLIGHT && PME_FLIGHT_AERODYNAMICS
pme_result YMPME_Flight_AeroLoad(const pme_aero_desc *a, pme_vec3 velocity, float density,
                                 pme_flight_load *out)
{
	double dynamicArea;
	float speed, drag, lift, length;
	pme_vec3 direction, liftDirection, force;
	if (!a || !out || !YMPME_VecFinite(velocity) || !YMPME_VecFinite(a->position) ||
	    !YMPME_VecFinite(a->liftAxis) || !isfinite(density) || density < 0 || !isfinite(a->area) ||
	    a->area < 0 || !isfinite(a->liftCoefficient) || !isfinite(a->dragCoefficient) ||
	    a->dragCoefficient < 0)
		return PME_INVALID_ARGUMENT;
	speed = hypotf(hypotf(velocity.x, velocity.y), velocity.z);
	if (!isfinite(speed))
		return PME_INVALID_ARGUMENT;
	if (speed < 1e-12f || density == 0 || a->area == 0)
	{
		*out = YMPME_Flight_ZeroLoad();
		return PME_OK;
	}
	direction = YMPME_VecScale(velocity, 1 / speed);
	dynamicArea = .5 * (double)density * speed * speed * a->area;
	drag = (float)(dynamicArea * a->dragCoefficient);
	lift = (float)(dynamicArea * a->liftCoefficient);
	if (!isfinite(drag) || !isfinite(lift))
		return PME_INVALID_ARGUMENT;
	force = YMPME_VecScale(direction, -drag);
	if (lift != 0)
	{
		length = hypotf(hypotf(a->liftAxis.x, a->liftAxis.y), a->liftAxis.z);
		if (!isfinite(length) || length < 1e-12f)
			return PME_INVALID_ARGUMENT;
		liftDirection = YMPME_VecScale(a->liftAxis, 1 / length);
		liftDirection =
		    YMPME_VecSub(liftDirection, YMPME_VecScale(direction, YMPME_VecDot(liftDirection, direction)));
		length = hypotf(hypotf(liftDirection.x, liftDirection.y), liftDirection.z);
		if (length < 1e-6f)
			return PME_INVALID_ARGUMENT;
		force = YMPME_VecAdd(force, YMPME_VecScale(liftDirection, lift / length));
	}
	return YMPME_Flight_LoadAtPoint(force, a->position, YMPME_Vec3(0, 0, 0), out);
}
#endif
