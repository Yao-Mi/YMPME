#include "YMPME_Creat.h"
#include "../CONFIG/YMPME_Mem.h"
PME_World YMPME_Create_World(size_t capacity, const pme_world_desc* desc)
{
	PME_World w;
	PME_Body bodies;
	if (!capacity || capacity > SIZE_MAX/sizeof(pme_body)) return NULL;
	w = PME_malloc0(sizeof(*w));
	if (!w) return NULL;
	bodies = PME_malloc0(capacity*sizeof(*bodies));
	if (!bodies || YMPME_World_Init(w, bodies, capacity, desc) != PME_OK)
	{
		PME_free0(bodies);
		PME_free0(w);
		return NULL;
	}
	w->ownsMemory = 1;
	return w;
}
