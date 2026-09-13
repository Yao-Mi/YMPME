#ifndef HUMAN_SKIN_OBSERVATION_H
#define HUMAN_SKIN_OBSERVATION_H
#include "CORE/YMPME_Surface.h"
enum
{
	HUMAN_OBSERVATION_SIZE = 512,
	HUMAN_GRID_OBSERVATION_SIZE = 64,
	HUMAN_OBSERVATION_EMPTY = 65535
};
struct human_skin_rig_;
/* Stable bind-pose mapping shared by human_skin and human_ground. 512x512 row-major,
 * top-left origin. IDs index forces/patches/triangles; EMPTY pixels have RGB (0,0,0).
 * RGB is Fn/Fu/Fv, uint8. Unloaded valid skin = (0,128,128). Float local is (Fu,Fv,Fn) N.
 * Same explicit fixed range for every face. No grid, selection, zoom, camera, gain or
 * color-mode dependence. Returns current simulation timestamp, without advancing time.
 * Caller supplies RGB (512*512*3 bytes) and local (faceCount vec3s), no allocations.
 * Adapts the demo's PME/SKE data; this header has no GRE/SKE types. */
const uint16_t *HumanSkin_ObservationMap(const struct human_skin_rig_ *s);
int HumanSkin_Observe(const struct human_skin_rig_ *s, uint8_t *rgb, size_t rgbCapacity,
					  pme_vec3 *local, size_t faceCapacity, pme_sensor_stamp *stamp);
/* Native topology observation: 64x64, exactly one pixel per face, no interpolation.
 * Each of the faceCount IDs occurs once; spare cells are EMPTY. Same force basis and
 * explicit ranges as the legacy atlas. UI layout switching cannot change this output.
 * Grid neighbours can differ from physical neighbours: use the adjacency accessor.
 */
const uint16_t *HumanSkin_GridObservationMap(const struct human_skin_rig_ *s);
/* 64x64 float m^2, row-major top-left, exactly aligned with GridObservationMap.
 * Empty cells = 0; occupied cells = initial-pose whole-triangle area (not the
 * actual contact footprint). Cached once during Init, ignores skin deformation.
 * Owned by rig until Destroy; values stay fixed across pose and display changes.
 * No allocation or geometry computation when reading.
 * Average pressure = Fn / area; guard empty cells. NULL rig returns NULL. */
const float *HumanSkin_GridAreas(const struct human_skin_rig_ *s);
const int32_t *HumanSkin_GridNeighbours(const struct human_skin_rig_ *s);
int HumanSkin_ObserveGrid(const struct human_skin_rig_ *s, uint8_t *rgb, size_t rgbCapacity,
						  pme_vec3 *local, size_t faceCapacity, pme_sensor_stamp *stamp);
int HumanSkin_ExportObservation(const struct human_skin_rig_ *s, const char *directory);
#endif
