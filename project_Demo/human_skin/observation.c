#include "rig.h"
#include "observation.h"
#include <stdio.h>
const uint16_t *HumanSkin_ObservationMap(const human_skin_rig *s)
{
	return s ? s->contactView.observationMap : NULL;
}
const uint16_t *HumanSkin_GridObservationMap(const human_skin_rig *s)
{
	return s ? s->contactView.gridIds : NULL;
}
const float *HumanSkin_GridAreas(const human_skin_rig *s)
{
	return s ? s->contactView.gridAreas : NULL;
}
const int32_t *HumanSkin_GridNeighbours(const human_skin_rig *s)
{
	return s ? s->contactView.neighbours : NULL;
}
static int observe(const human_skin_rig *s, uint8_t *rgb, size_t capacity, pme_vec3 *local,
				   size_t faces, pme_sensor_stamp *stamp, int grid)
{
	size_t count = grid ? HUMAN_GRID_CELLS : HUMAN_OBSERVATION_SIZE * HUMAN_OBSERVATION_SIZE;
	if (!s || !rgb || !local || !stamp || capacity < count * 3 || faces < s->count)
		return 0;
	GRErgb24 colors[DEMO_FACES];
	for (size_t p = 0; p < s->count; p++)
	{
		pme_vec3 tri[3];
		for (int k = 0; k < 3; k++)
		{
			gre_fvector4d v = s->human.object->pointList[s->triangle[p][k]].pos;
			tri[k] = YMPME_Vec3(v.x * .1f, v.y * .1f, v.z * .1f);
		}
		if (!DemoForce_Local(tri, s->contactView.densityCorners + p * 6, s->patches[p].normal,
							 s->forces[p], local + p))
			return 0;
		colors[p] = DemoForce_RGB(local[p], s->contactView.observationNormalRange,
								  s->contactView.observationTangentRange, NULL);
	}
	for (size_t i = 0; i < count; i++)
	{
		uint16_t p = grid ? s->contactView.gridIds[i] : s->contactView.observationMap[i];
		GRErgb24 c = p == UV_EMPTY ? (GRErgb24){0, 0, 0} : colors[p];
		rgb[i * 3] = c.R;
		rgb[i * 3 + 1] = c.G;
		rgb[i * 3 + 2] = c.B;
	}
	*stamp = s->touch.stamp;
	return 1;
}
int HumanSkin_Observe(const human_skin_rig *s, uint8_t *rgb, size_t capacity, pme_vec3 *local,
					  size_t faces, pme_sensor_stamp *stamp)
{
	return observe(s, rgb, capacity, local, faces, stamp, 0);
}
int HumanSkin_ObserveGrid(const human_skin_rig *s, uint8_t *rgb, size_t capacity, pme_vec3 *local,
						  size_t faces, pme_sensor_stamp *stamp)
{
	return observe(s, rgb, capacity, local, faces, stamp, 1);
}
static int exportGrid(const human_skin_rig *s, const char *dir)
{
	uint8_t rgb[HUMAN_GRID_CELLS * 3];
	pme_vec3 local[DEMO_FACES];
	pme_sensor_stamp stamp;
	if (!HumanSkin_ObserveGrid(s, rgb, sizeof rgb, local, DEMO_FACES, &stamp))
		return 0;
	const char *names[] = {"skin_grid_rgb.ppm", "skin_grid_face_ids.pgm", "skin_grid_metadata.json",
						   "skin_grid_adjacency.csv", "skin_grid_area_m2.csv"};
	for (int k = 0; k < 5; k++)
	{
		char path[4096];
		int n = snprintf(path, sizeof path, "%s/%s", dir, names[k]);
		if (n < 0 || (size_t)n >= sizeof path)
			return 0;
		FILE *f = fopen(path, "wb");
		if (!f)
			return 0;
		int ok = 1;
		if (k == 0)
		{
			fprintf(f, "P6\n64 64\n255\n");
			ok = fwrite(rgb, 1, sizeof rgb, f) == sizeof rgb;
		}
		if (k == 1)
		{
			fprintf(f, "P5\n64 64\n65535\n");
			for (size_t i = 0; i < HUMAN_GRID_CELLS; i++)
			{
				uint16_t id = s->contactView.gridIds[i];
				if (fputc(id >> 8, f) == EOF || fputc(id & 255, f) == EOF)
				{
					ok = 0;
					break;
				}
			}
		}
		if (k == 2)
			fprintf(
				f,
				"{\"schema\":\"human-topology-grid-v1\",\"width\":64,\"height\":64,\"faces\":%zu,"
				"\"one_cell_per_face\":true,\"time_s\":%.9g,\"normal_range_N\":%.9g,\"tangent_"
				"range_N\":%.9g,\"empty_id\":65535,\"channels\":[\"Fn\",\"Fu\",\"Fv\"],\"origin\":"
				"\"top-left\",\"mapping_hash\":\"%016llx\",\"mesh_id\":\"%016llx\",\"basis\":"
				"\"fixed triangle atlas "
				"tangents\",\"area_file\":\"skin_grid_area_m2.csv\",\"area_unit\":\"m^2\",\"area_"
				"definition\":\"initial-pose whole "
				"triangle\",\"area_empty\":0,\"adjacent_mean_cells\":%.6g,\"adjacent_within_two_"
				"fraction\":%.6g}\n",
				s->count, s->time, s->contactView.observationNormalRange,
				s->contactView.observationTangentRange,
				(unsigned long long)s->contactView.gridResult.mappingHash,
				(unsigned long long)s->human.asset.info.meshId,
				s->contactView.gridResult.meanDistance,
				s->contactView.gridResult.withinTwoFraction);
		if (k == 3)
		{
			fprintf(f, "face,cell_x,cell_y,neighbour0,neighbour1,neighbour2\n");
			for (size_t i = 0; i < s->count; i++)
				fprintf(f, "%zu,%u,%u,%d,%d,%d\n", i, s->contactView.faceCells[i] % 64,
						s->contactView.faceCells[i] / 64, s->contactView.neighbours[i * 3],
						s->contactView.neighbours[i * 3 + 1], s->contactView.neighbours[i * 3 + 2]);
		}
		if (k == 4)
			for (size_t y = 0; y < HUMAN_GRID_SIDE; y++)
				for (size_t x = 0; x < HUMAN_GRID_SIDE; x++)
					fprintf(f, x + 1 == HUMAN_GRID_SIDE ? "%.9g\n" : "%.9g,",
							s->contactView.gridAreas[y * HUMAN_GRID_SIDE + x]);
		if (ferror(f))
			ok = 0;
		if (fclose(f) != 0)
			ok = 0;
		if (!ok)
			return 0;
	}
	return 1;
}
/* Explicit export only, never in the realtime path. Caller creates the directory. */
int HumanSkin_ExportObservation(const human_skin_rig *s, const char *dir)
{
	uint8_t rgb[HUMAN_OBSERVATION_SIZE * HUMAN_OBSERVATION_SIZE * 3];
	pme_vec3 local[DEMO_FACES];
	pme_sensor_stamp stamp;
	if (!dir || !HumanSkin_Observe(s, rgb, sizeof rgb, local, DEMO_FACES, &stamp))
		return 0;
	const char *names[] = {"skin_rgb.ppm", "skin_face_ids.pgm", "skin_forces.csv",
						   "skin_metadata.json"};
	for (int k = 0; k < 4; k++)
	{
		char path[4096];
		int n = snprintf(path, sizeof path, "%s/%s", dir, names[k]);
		if (n < 0 || (size_t)n >= sizeof path)
			return 0;
		FILE *f = fopen(path, "wb");
		if (!f)
			return 0;
		int ok = 1;
		if (k == 0)
		{
			fprintf(f, "P6\n512 512\n255\n");
			ok = fwrite(rgb, 1, sizeof rgb, f) == sizeof rgb;
		}
		if (k == 1)
		{
			fprintf(f, "P5\n512 512\n65535\n");
			for (size_t i = 0; i < 512 * 512; i++)
			{
				uint16_t id = s->contactView.observationMap[i];
				if (fputc(id >> 8, f) == EOF || fputc(id & 255, f) == EOF)
				{
					ok = 0;
					break;
				}
			}
		}
		if (k == 2)
		{
			fprintf(
				f,
				"face,vertex0,vertex1,vertex2,Fn_N,Fu_N,Fv_N,Fx_N,Fy_N,Fz_N,area_m2,x_m,y_m,z_m\n");
			for (size_t p = 0; p < s->count; p++)
				fprintf(f, "%zu,%u,%u,%u,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g\n", p,
						s->triangle[p][0], s->triangle[p][1], s->triangle[p][2], local[p].z,
						local[p].x, local[p].y, s->forces[p].x, s->forces[p].y, s->forces[p].z,
						s->patches[p].area, s->patches[p].position.x, s->patches[p].position.y,
						s->patches[p].position.z);
		}
		if (k == 3)
			fprintf(f,
					"{\"schema\":\"human-skin-observation-v1\",\"width\":512,\"height\":512,"
					"\"faces\":%zu,\"time_s\":%.9g,\"normal_range_N\":%.9g,\"tangent_range_N\":%."
					"9g,\"empty_id\":65535,\"channels\":[\"Fn\",\"Fu\",\"Fv\"],\"origin\":\"top-"
					"left\",\"id_encoding\":\"PGM big-endian uint16\",\"mesh_id\":\"%016llx\"}\n",
					s->count, s->time, s->contactView.observationNormalRange,
					s->contactView.observationTangentRange,
					(unsigned long long)s->human.asset.info.meshId);
		if (ferror(f))
			ok = 0;
		if (fclose(f) != 0)
			ok = 0;
		if (!ok)
			return 0;
	}
	return exportGrid(s, dir);
}
