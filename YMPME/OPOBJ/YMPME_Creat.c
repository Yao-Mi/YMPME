#include "YMPME_Creat.h"
#include "../DEBUG/YMPME_Log.h"
#include "../CONFIG/YMPME_Mem.h"
PME_World YMPME_Create_World(size_t capacity, const pme_world_desc* desc)
{
	PME_World w;
	PME_Body bodies;
	if (!capacity || capacity > SIZE_MAX/sizeof(pme_body)) {
		YMPME_LOG(PME_LOG_ERROR, PME_INVALID_ARGUMENT, "Invalid world allocation capacity");
		return NULL;
	}
	w = PME_malloc0(sizeof(*w));
	if (!w) {
		YMPME_LOG(PME_LOG_ERROR, PME_OUT_OF_MEMORY, "World allocation failed");
		return NULL;
	}
	bodies = PME_malloc0(capacity*sizeof(*bodies));
	if (!bodies) YMPME_LOG(PME_LOG_ERROR, PME_OUT_OF_MEMORY, "Body storage allocation failed");
	if (!bodies || YMPME_World_Init(w, bodies, capacity, desc) != PME_OK)
	{
		PME_free0(bodies);
		PME_free0(w);
		return NULL;
	}
	w->ownsMemory = 1;
	return w;
}
