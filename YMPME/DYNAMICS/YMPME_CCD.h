#ifndef YMPME_CCD_INTERNAL_H
#define YMPME_CCD_INTERNAL_H
#include "YMPME_Dynamics.h"
/* Called only after ordinary body/joint/collider validation. No allocation. */
pme_result PME_CCD_Validate(const pme_dynamics *world);
void PME_CCD_Integrate(pme_dynamics *world, size_t body, float dt, double endTime);
#endif
