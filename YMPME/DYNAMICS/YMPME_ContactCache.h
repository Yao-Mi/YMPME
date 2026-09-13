#ifndef YMPME_CONTACT_CACHE_H
#define YMPME_CONTACT_CACHE_H
#include "YMPME_Dynamics.h"
int PME_ContactCache_Valid(const pme_contact_cache *cache);
void PME_ContactCache_WarmStart(pme_dynamics *world, float dt);
void PME_ContactCache_Store(pme_dynamics *world, float dt, double time);
#endif
