#ifndef HUMAN_CONTACT_VIEW_H
#define HUMAN_CONTACT_VIEW_H
#include "uv.h"
#include "MODEL/SURFACE_ATLAS/YMPME_SurfaceAtlas.h"
#include "MODEL/SURFACE_ATLAS/YMPME_TopologyGrid.h"
enum
{
	HUMAN_GRID_SIDE = 64,
	HUMAN_GRID_CELLS = 64 * 64
};
typedef struct human_contact_view_
{
	human_uv_map density;
	uint32_t gridCells[HUMAN_GRID_CELLS], faceCells[UV_MAX_FACES];
	uint16_t gridIds[HUMAN_GRID_CELLS];
	float gridAreas[HUMAN_GRID_CELLS]; /* fixed initial whole-face area, m^2; empty = 0 */
	int32_t neighbours[UV_MAX_FACES * 3];
	pme_topology_grid_result gridResult;
	uint8_t groups[UV_MAX_FACES];
	int showGroups;
	pme_surface_atlas_result atlas;
	float densityCorners[UV_MAX_FACES * 6];
	uint16_t observationMap[UV_VIEW * UV_VIEW];
	float observationNormalRange, observationTangentRange;
	int overview; /* 1: fixed PME atlas, 2: GRE proportion-preserving atlas, 3: topology grid */
} human_contact_view;
struct human_skin_rig_;
int HumanContactView_Init(struct human_skin_rig_ *s);
void HumanContactView_Edges(const struct human_skin_rig_ *s, GRE_RenderTarget target, int region,
							float centerU, float centerV, float zoom);
int HumanContactView_GridPick(const struct human_skin_rig_ *s, float u, float v);
void HumanContactView_GridRender(const struct human_skin_rig_ *s, GRE_RenderTarget target);
const char *HumanContactView_GroupName(const struct human_skin_rig_ *s, int face);
void HumanContactView_FocusFace(struct human_skin_rig_ *s, int face);
void HumanContactView_Focus(struct human_skin_rig_ *s, int region);
#endif
