#ifndef HUMAN_SKIN_UV_H
#define HUMAN_SKIN_UV_H
#include "YMGRE_Rendering_Pipeline.h"
enum
{
	UV_SIZE = 2048,
	UV_VIEW = 512,
	UV_EMPTY = UINT16_MAX,
	UV_MAX_FACES = 3584
};
typedef struct human_uv_map_
{
	uint16_t patch[UV_SIZE * UV_SIZE];
	uint8_t interior[UV_SIZE * UV_SIZE], overlap[UV_SIZE * UV_SIZE];
	size_t count, overlapPixels, uncoveredPatches;
} human_uv_map;
int HumanUV_Init(human_uv_map *map, const float *corners, size_t count);
int HumanUV_Pick(const human_uv_map *map, int x, int y);
void HumanUV_Render(const human_uv_map *map, GRE_RenderTarget target, const GRErgb24 *colors,
					int selected, float centerU, float centerV, float zoom);
#endif
