#include "YMPME_Free.h"
#include "../CONFIG/YMPME_Mem.h"
void YMPME_Free_World(PME_World w)
{
	if (!w || !w->ownsMemory) return;
	PME_free0(w->bodies);
	PME_free0(w);
}
