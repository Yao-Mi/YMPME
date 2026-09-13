/* 公共值类型；时间单位秒，长度单位米，质量单位千克。 */
#ifndef YMPME_PUBTYPE_H
#define YMPME_PUBTYPE_H
#include <stddef.h>
#include <stdint.h>
typedef struct pme_vec3_ { float x, y, z; } pme_vec3;
typedef enum pme_result_
{
	PME_OK = 0,
	PME_INVALID_ARGUMENT = -1,
	PME_CAPACITY_EXCEEDED = -2
} pme_result;
#endif
