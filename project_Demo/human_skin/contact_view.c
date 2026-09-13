#include "rig.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
int HumanContactView_Init(human_skin_rig *s)
{
	human_contact_view *v = &s->contactView;
	pme_vec3 positions[DEMO_POINTS];
	uint32_t indices[DEMO_FACES * 3], ids[DEMO_FACES], canonical[DEMO_POINTS], queue[DEMO_FACES];
	pme_surface_atlas_edge edges[DEMO_FACES * 3];
	pme_surface_atlas_face faces[DEMO_FACES];
	pme_surface_atlas_chart charts[DEMO_FACES];
	pme_surface_atlas_workspace work = {canonical, queue,		edges,	   faces,
										charts,	   DEMO_POINTS, DEMO_FACES};
	pme_surface_atlas_desc desc = YMPME_SurfaceAtlas_DefaultDesc();
	desc.padding = .04f;
	for (unsigned i = 0; i < s->human.asset.info.vertexCount; i++)
	{
		ske_vec3 p = s->human.vertices[i].position;
		positions[i] = YMPME_Vec3(p.x * .1f, p.y * .1f, p.z * .1f);
	}
	for (size_t p = 0; p < s->count; p++)
		for (int k = 0; k < 3; k++)
			indices[p * 3 + k] = s->triangle[p][k];
	if (YMPME_SurfaceAtlas_Build(&desc, positions, s->human.asset.info.vertexCount, indices,
								 s->count, NULL, &work, v->densityCorners, ids, DEMO_FACES,
								 &v->atlas) != PME_OK)
		return 0;
	for (size_t f = 0; f < s->count; f++)
		for (int k = 0; k < 3; k++)
			v->neighbours[f * 3 + k] = work.faces[f].neighbour[k];
	float xy[DEMO_FACES * 2], next[DEMO_FACES * 2];
	pme_topology_grid_workspace gridWork = {xy, next, queue, DEMO_FACES};
	pme_topology_grid_desc gridDesc = YMPME_TopologyGrid_DefaultDesc();
	if (YMPME_TopologyGrid_Build(&gridDesc, v->neighbours, s->count, &gridWork, v->gridCells,
								 HUMAN_GRID_CELLS, v->faceCells, DEMO_FACES,
								 &v->gridResult) != PME_OK)
		return 0;
	/* Colours are diagnostics only; weights/regions never enter the grid optimizer. */
	for (size_t f = 0; f < s->count; f++)
	{
		float votes[10] = {0};
		for (int k = 0; k < 3; k++)
			for (int j = 0; j < YMSKE_MAX_INFLUENCES; j++)
			{
				const ske_weights *w = &s->human.vertices[s->triangle[f][k]].weights;
				unsigned b = w->bone[j];
				int g = (b == 20 || b == 21)   ? 1
						: (b >= 22 && b <= 24) ? 2
						: (b >= 25 && b <= 39) ? 3
						: (b >= 40 && b <= 42) ? 4
						: (b >= 43 && b <= 57) ? 5
						: (b >= 12 && b <= 14) ? 6
						: (b == 15 || b == 16) ? 7
						: (b >= 58 && b <= 60) ? 8
						: (b == 61 || b == 62) ? 9
											   : 0;
				votes[g] += w->weight[j];
			}
		int best = 0;
		for (int g = 1; g < 10; g++)
			if (votes[g] > votes[best])
				best = g;
		v->groups[f] = (uint8_t)best;
	}
	uint8_t unique[DEMO_FACES] = {0};
	for (size_t c = 0; c < HUMAN_GRID_CELLS; c++)
	{
		uint32_t f = v->gridCells[c];
		v->gridIds[c] = f == PME_TOPOLOGY_EMPTY ? UV_EMPTY : (uint16_t)f;
		v->gridAreas[c] = 0;
		if (f != PME_TOPOLOGY_EMPTY)
		{
			if (f >= s->count || unique[f]++ || v->faceCells[f] != c)
				return 0;
			v->gridAreas[c] = s->patches[f].area;
		}
	}
	for (size_t f = 0; f < s->count; f++)
		if (unique[f] != 1)
			return 0;
	printf("Topology grid: %zu unique faces / %zu cells, edge distance %.2f -> %.2f cells, %.1f%% "
		   "within 2, hash %016llx\n",
		   s->count, v->gridResult.cells, v->gridResult.initialMeanDistance,
		   v->gridResult.meanDistance, v->gridResult.withinTwoFraction * 100,
		   (unsigned long long)v->gridResult.mappingHash);
	float oldOccupancy = v->atlas.occupancy;
	uint64_t occupied[960 * 15], shape[960 * 15];
	if (YMPME_SurfaceAtlas_Compact(&desc, &work, s->count, 960, 12, occupied, shape, 960 * 15,
								   v->densityCorners, &v->atlas) != PME_OK ||
		!HumanUV_Init(&v->density, v->densityCorners, s->count) || v->density.overlapPixels ||
		v->density.uncoveredPatches)
		return 0;
	printf("Compact sensor atlas occupancy %.1f%% -> %.1f%%\n", oldOccupancy * 100,
		   v->atlas.occupancy * 100);
	uint8_t seen[DEMO_FACES] = {0};
	for (int y = 0; y < UV_VIEW; y++)
		for (int x = 0; x < UV_VIEW; x++)
		{
			int p = HumanUV_Pick(&v->density, x * (UV_SIZE / UV_VIEW) + UV_SIZE / UV_VIEW / 2,
								 y * (UV_SIZE / UV_VIEW) + UV_SIZE / UV_VIEW / 2);
			v->observationMap[y * UV_VIEW + x] = p < 0 ? UV_EMPTY : (uint16_t)p;
			if (p >= 0)
				seen[p] = 1;
		}
	for (size_t p = 0; p < s->count; p++)
		if (!seen[p])
		{
			fprintf(stderr, "Full-body observation missing face %zu\n", p);
			return 0;
		}
	v->overview = 3;
	v->observationNormalRange = .1f;
	v->observationTangentRange = .05f;
	printf("PME full-body atlas: %zu faces / %zu charts, all faces covered at 512x512\n", s->count,
		   v->atlas.charts);
	return 1;
}
static void edge(GRE_RenderTarget t, int x0, int y0, int x1, int y1)
{
	int dx = abs(x1 - x0), dy = -abs(y1 - y0), sx = x0 < x1 ? 1 : -1, sy = y0 < y1 ? 1 : -1,
		error = dx + dy;
	for (;;)
	{
		if (x0 >= 0 && y0 >= 0 && x0 < t->width && y0 < t->height)
			t->data[y0 * t->width + x0] = GRE_FramePixel_From_RGB24((GRErgb24){36, 63, 70});
		if (x0 == x1 && y0 == y1)
			break;
		int twice = error * 2;
		if (twice >= dy)
		{
			error += dy;
			x0 += sx;
		}
		if (twice <= dx)
		{
			error += dx;
			y0 += sy;
		}
	}
}
void HumanContactView_Edges(const human_skin_rig *s, GRE_RenderTarget t, int region, float u,
							float v, float zoom)
{
	for (size_t p = 0; p < s->count; p++)
		if (!region || s->self.faces[p].region == region)
		{
			const float *uv = s->contactView.densityCorners + p * 6;
			int x[3], y[3];
			for (int k = 0; k < 3; k++)
			{
				x[k] = (int)lroundf(((uv[k * 2] - u) * zoom + .5f) * t->width);
				y[k] = (int)lroundf(((uv[k * 2 + 1] - v) * zoom + .5f) * t->height);
			}
			for (int k = 0; k < 3; k++)
				edge(t, x[k], y[k], x[(k + 1) % 3], y[(k + 1) % 3]);
		}
}

void HumanContactView_Focus(human_skin_rig *s, int region)
{
	int peak = -1;
	for (size_t p = 0; p < s->count; p++)
		if (s->self.faces[p].region == region &&
			(peak < 0 || YMPME_VecDot(s->forces[p], s->forces[p]) >
							 YMPME_VecDot(s->forces[peak], s->forces[peak])))
			peak = (int)p;
	if (peak < 0)
		return;
	HumanContactView_FocusFace(s, peak);
}
void HumanContactView_FocusFace(human_skin_rig *s, int face)
{
	if (face < 0 || (size_t)face >= s->count)
		return;
	s->selectedUv = face;
	s->uvZoom = 4;
	float u, v;
	if (s->contactView.overview == 3)
	{
		uint32_t c = s->contactView.faceCells[face];
		u = (c % HUMAN_GRID_SIDE + .5f) / HUMAN_GRID_SIDE;
		v = (c / HUMAN_GRID_SIDE + .5f) / HUMAN_GRID_SIDE;
	}
	else
	{
		s->contactView.overview = 1;
		const float *uv = s->contactView.densityCorners + face * 6;
		u = (uv[0] + uv[2] + uv[4]) / 3;
		v = (uv[1] + uv[3] + uv[5]) / 3;
	}
	s->uvCenterU = YMPME_Clamp(u, .125f, .875f);
	s->uvCenterV = YMPME_Clamp(v, .125f, .875f);
}
int HumanContactView_GridPick(const human_skin_rig *s, float u, float v)
{
	if (!isfinite(u + v) || u < 0 || v < 0 || u >= 1 || v >= 1)
		return -1;
	uint32_t f = s->contactView.gridCells[(unsigned)(v * HUMAN_GRID_SIDE) * HUMAN_GRID_SIDE +
										  (unsigned)(u * HUMAN_GRID_SIDE)];
	return f == PME_TOPOLOGY_EMPTY ? -1 : (int)f;
}
void HumanContactView_GridRender(const human_skin_rig *s, GRE_RenderTarget target)
{
	for (int y = 0; y < target->height; y++)
		for (int x = 0; x < target->width; x++)
		{
			float u = s->uvCenterU + ((x + .5f) / target->width - .5f) / s->uvZoom;
			float v = s->uvCenterV + ((y + .5f) / target->height - .5f) / s->uvZoom;
			int p = HumanContactView_GridPick(s, u, v);
			GRErgb24 c = p < 0 ? (GRErgb24){15, 21, 31} : s->colors[p];
			if (p >= 0 && s->contactView.showGroups)
			{
				static const GRErgb24 palette[10] = {
					{159, 176, 192}, {244, 208, 99}, {121, 98, 233}, {93, 144, 248},
					{240, 123, 75},	 {238, 94, 144}, {71, 199, 136}, {112, 229, 209},
					{169, 207, 80},	 {222, 180, 85}};
				c = palette[s->contactView.groups[p]];
			}

			if (p >= 0)
			{
				float gx = u * HUMAN_GRID_SIDE, gy = v * HUMAN_GRID_SIDE;
				float edgeU = fminf(gx - floorf(gx), ceilf(gx) - gx),
					  edgeV = fminf(gy - floorf(gy), ceilf(gy) - gy);
				float border = (float)HUMAN_GRID_SIDE / (target->width * s->uvZoom) * .55f;
				if (edgeU < border || edgeV < border)
					c = p == s->selectedUv						  ? (GRErgb24){255, 245, 180}
						: HumanSkin_IsContactMarked(s, (size_t)p) ? (GRErgb24){245, 245, 245}
																  : (GRErgb24){36, 63, 70};
			}
			target->data[y * target->width + x] = GRE_FramePixel_From_RGB24(c);
		}
}

const char *HumanContactView_GroupName(const human_skin_rig *s, int face)
{
	static const char *names[] = {"TORSO",	"HEAD",	 "ARM L",  "HAND L", "ARM R",
								  "HAND R", "LEG L", "FOOT L", "LEG R",	 "FOOT R"};
	return face >= 0 && (size_t)face < s->count ? names[s->contactView.groups[face]] : "NONE";
}
