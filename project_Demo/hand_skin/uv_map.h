#ifndef HAND_UV_MAP_H
#define HAND_UV_MAP_H
#include "scene.h"
#include "SENSOR/TACTILE/YMPME_SurfaceTactile.h"
enum
{
	HAND_UV_SIZE = 512,
	HAND_UV_EMPTY = UINT16_MAX
};
typedef struct hand_uv_map_
{
	uint16_t patch[HAND_UV_SIZE * HAND_UV_SIZE];
	uint8_t interior[HAND_UV_SIZE * HAND_UV_SIZE], overlap[HAND_UV_SIZE * HAND_UV_SIZE];
	unsigned face[HAND_FACES];
	size_t count, overlapPixels, uncoveredPatches;
} hand_uv_map;
/* UV由GRE资源提供：u向右、v向下，与现有BMP纹理采样相同，不重复翻转。 */
int HandUV_Init(hand_uv_map *map, const unsigned *faces, size_t count);
/* 每个有效patch三个角点的(u,v)，顺序与faces一致；不改变蒙皮顶点。 */
int HandUV_InitCorners(hand_uv_map *map, const unsigned *faces, size_t count,
					   const float *cornerUv);
int HandUV_Pick(const hand_uv_map *map, int x, int y); /* -1空白，-2重叠歧义。 */
GRErgb24 HandUV_Color(float value, float maximum);
void HandUV_RenderColors(const hand_uv_map *map, GRE_RenderTarget target, const GRErgb24 *colors,
						 int selected);
void HandUV_Render(const hand_uv_map *map, GRE_RenderTarget target, const pme_surface_taxel *cells,
				   int forceMode, int selected);
#endif
