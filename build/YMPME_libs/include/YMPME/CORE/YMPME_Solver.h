/* 内部求解器；接触法线从 A 指向 B。 */
#ifndef YMPME_SOLVER_H
#define YMPME_SOLVER_H
#include "YMPME_World.h"
void YMPME_Solver_Resolve(PME_Body a, PME_Body b, const pme_contact* c, const pme_world_desc* config);
void YMPME_Solver_ResolveFeedback(PME_Body a, PME_Body b, const pme_contact* c,
 const pme_world_desc* config,pme_contact_feedback* feedback);
#endif
