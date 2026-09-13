#include "scene.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define CHECK(x)                                                                                   \
	do                                                                                             \
	{                                                                                              \
		if (!(x))                                                                                  \
		{                                                                                          \
			fprintf(stderr, "line %d: %s\n", __LINE__, #x);                                        \
			exit(1);                                                                               \
		}                                                                                          \
	} while (0)
typedef struct counter_
{
	pme_ray_provider source;
	size_t calls;
} counter;
static pme_result counted(void *context, const pme_ray *ray, pme_ray_hit *hit)
{
	counter *c = context;
	++c->calls;
	return c->source.cast(c->source.context, ray, hit);
}
static uint64_t hash(GRE_RenderTarget t)
{
	const unsigned char *p = (const unsigned char *)t->data;
	uint64_t h = 14695981039346656037ULL;
	for (size_t i = 0; i < (size_t)t->width * t->height * sizeof(*t->data); ++i)
		h = (h ^ p[i]) * 1099511628211ULL;
	return h;
}
int main(void)
{
	static sensor_scene s;
	static sensor_rig snapshot;
	gre_line3d lines[4];
	sensor_lidar_segment segment = {{1, 2, 3}, {4, 5, 6}, 1};
	CHECK(SensorLidar_BeamLines(&segment, SENSOR_LIDAR_OFF, 2, lines) == 0);
	CHECK(SensorLidar_BeamLines(&segment, SENSOR_LIDAR_POINTS, 2, lines) == 3);
	CHECK(SensorLidar_BeamLines(&segment, SENSOR_LIDAR_LINES, 2, lines) == 1);
	CHECK(lines[0].start.x == 2 && lines[0].start.y == 4 && lines[0].end.x == 8 &&
		  lines[0].end.z == 12);
	CHECK(SensorLidar_BeamLines(&segment, SENSOR_LIDAR_BOTH, 2, lines) == 4);
	segment.hit = 0;
	CHECK(SensorLidar_BeamLines(&segment, SENSOR_LIDAR_POINTS, 2, lines) == 0);
	CHECK(SensorLidar_BeamLines(&segment, SENSOR_LIDAR_BOTH, 2, lines) == 1);
	CHECK(SensorScene_Init(&s));
	counter count = {s.rig.provider, 0};
	s.rig.provider = (pme_ray_provider){counted, &count};
	s.rig.shortRange = 1;
	s.rig.noise = 1;
	CHECK(SensorRig_Sample(&s.rig));
	CHECK(count.calls == LAB_BEAMS + LAB_DEPTH_WIDTH * LAB_DEPTH_HEIGHT + PME_SENSOR_LASER);
	CHECK(s.rig.lidarTraceCount == LAB_BEAMS);
	size_t misses = 0, hits = 0;
	for (size_t i = 0; i < LAB_BEAMS; ++i)
	{
		const sensor_lidar_segment *beam = &s.rig.lidarTrace[i];
		pme_vec3 d = YMPME_VecSub(beam->end, beam->origin);
		CHECK(YMPME_VecDot(YMPME_VecSub(beam->origin, s.rig.scan.origin),
						   YMPME_VecSub(beam->origin, s.rig.scan.origin)) < 1e-10f);
		if (!beam->hit)
		{
			++misses;
			CHECK(fabsf(sqrtf(YMPME_VecDot(d, d)) - 3) < 1e-5f);
		}
		else
			++hits;
	}
	CHECK(misses && hits);
	/* Disable valid measurements with an oversized blind zone: geometric rays
	 * still stop at the near obstacle, instead of incorrectly passing through. */
	s.rig.lidar.minRange = 2.99f;
	CHECK(SensorRig_Sample(&s.rig));
	int blind = 0;
	for (size_t i = 0; i < LAB_BEAMS; ++i)
		if (s.rig.lidarTrace[i].hit && !s.rig.points[i].valid)
			blind = 1;
	CHECK(blind);
	s.rig.lidar.minRange = .05f;
	CHECK(SensorRig_Sample(&s.rig));
	memcpy(&snapshot, &s.rig, sizeof snapshot);
	size_t casts = count.calls;
	uint64_t pictures[4];
	for (unsigned mode = 0; mode < 4; ++mode)
	{
		s.projection.mode = mode;
		s.projection.stride = 1;
		SensorScene_Render(&s, 0);
		pictures[mode] = hash(YMGRE_Camera_GetRenderTarget(s.camera));
		CHECK(!memcmp(&snapshot, &s.rig, sizeof snapshot) && count.calls == casts);
	}
	for (int i = 0; i < 4; ++i)
		for (int j = i + 1; j < 4; ++j)
			CHECK(pictures[i] != pictures[j]);
	s.projection.stride = 15;
	s.projection.depthTest = 0;
	SensorScene_Render(&s, 1);
	CHECK(count.calls == casts && !memcmp(&snapshot, &s.rig, sizeof snapshot));
	CHECK(SensorScene_CheckHandedness(&s));
	SensorScene_Destroy(&s);
	puts("LIDAR PROJECTION PASS: modes, hits/misses/blind zone, cached origins, no new ray casts "
		 "or sensor writes");
	return 0;
}
