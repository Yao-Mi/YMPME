#ifndef YMPME_MEM_H
#define YMPME_MEM_H
#include <stddef.h>
#ifdef __cplusplus
extern "C" {
#endif
/** PC 默认实现使用 malloc/free；MCU 可替换此 .c，或只使用 Init API。 */
void* PME_malloc0(size_t size);
void PME_free0(void* memory);
#ifdef __cplusplus
}
#endif
#endif
