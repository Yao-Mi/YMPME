#ifndef YMPME_CREAT_H
#define YMPME_CREAT_H
#include "../CORE/YMPME_World.h"
#ifdef __cplusplus
extern "C" {
#endif
/** 分配 world 和固定容量槽位；失败返回 NULL；用 YMPME_Free_World 释放。 */
PME_World YMPME_Create_World(size_t capacity, const pme_world_desc* desc);
#ifdef __cplusplus
}
#endif
#endif
