#ifndef YMPME_INDEX_H
#define YMPME_INDEX_H
#include <stdint.h>
#ifndef YMPME_INDEX_BITS
#define YMPME_INDEX_BITS 16
#endif
#if YMPME_INDEX_BITS == 16
typedef uint16_t PME_Index;
#define YMPME_INDEX_MAX UINT16_MAX
#elif YMPME_INDEX_BITS == 32
typedef uint32_t PME_Index;
#define YMPME_INDEX_MAX UINT32_MAX
#else
#error "YMPME_INDEX_BITS must be 16 or 32"
#endif
#endif
