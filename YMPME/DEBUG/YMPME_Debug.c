#include "YMPME_Debug.h"
double YMPME_Debug_KineticEnergy(const pme_world* w)
{
	double energy = 0;
	size_t i;
	if (!w || !w->bodies) return 0;
	for (i = 0; i < w->capacity; ++i)
	{
		const pme_body* b = &w->bodies[i];
		pme_vec3 v = b->state.velocity;
		if (b->active && b->state.type == PME_DYNAMIC)
			energy += 0.5*(double)b->state.mass*((double)v.x*v.x+(double)v.y*v.y+(double)v.z*v.z);
	}
	return energy;
}
