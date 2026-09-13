#ifndef YMPME_FREE_H
#define YMPME_FREE_H
#include "../CORE/YMPME_World.h"
#ifdef __cplusplus
extern "C" {
#endif
/** 只释放 Create 创建的世界；NULL 安全；不释放 body.userData。 */
void YMPME_Free_World(PME_World world);
#ifdef __cplusplus
}
#endif
#endif
