#include "YMPME_Mem.h"
#include <stdlib.h>
void* PME_malloc0(size_t size)
{
	return malloc(size);
}
void PME_free0(void* memory)
{
	free(memory);
}
