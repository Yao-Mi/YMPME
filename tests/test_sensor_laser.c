#include "SENSOR/LASER/YMPME_Laser.h"
#include "CONFIG/YMPME_Features.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#if !PME_SENSOR_LASER || PME_SENSOR_LIDAR || PME_SENSOR_DEPTH
#error "Standalone laser target must not pull in LiDAR/depth"
#endif
#define CHECK(x)                                                                                   \
	do                                                                                             \
	{                                                                                              \
		if (!(x))                                                                                  \
		{                                                                                          \
			fprintf(stderr, "line %d: %s\n", __LINE__, #x);                                        \
			exit(1);                                                                               \
		}                                                                                          \
	} while (0)
typedef struct fixture_
{
	int calls, hit, fail;
	float distance;
	pme_ray ray;
} fixture;
static pme_result cast(void *ctx, const pme_ray *ray, pme_ray_hit *hit)
{
	fixture *f = ctx;
	++f->calls;
	f->ray = *ray;
	if (f->fail)
		return PME_INVALID_ARGUMENT;
	*hit = (pme_ray_hit){f->hit, f->distance, {0, 0, 0}, {0, 0, -1}, 17, SIZE_MAX};
	return PME_OK;
}
int main(void)
{
	pme_laser_desc d = YMPME_Laser_DefaultDesc();
	pme_laser_sample out = {0}, old;
	pme_feedback_state s = {0};
	s.valid = PME_FEEDBACK_POSE;
	s.time = 1;
	s.rotation = YMPME_Rotation_Identity();
	s.position = YMPME_Vec3(1, 2, 3);
	pme_sensor_stamp stamp = {1, 1.1, 7, 1};
	uint32_t seed = 123;
	fixture f = {0, 1, 0, 5.1234f, {{0, 0, 0}, {0, 0, 0}, 0}};
	pme_ray_provider p = {cast, &f};
	CHECK(YMPME_Laser_Measure(&d, &s, &stamp, &p, &seed, &out) == PME_OK);
	CHECK(f.calls == 1 && out.valid && out.status == PME_LASER_VALID && out.body == 17);
	CHECK(fabsf(out.distance - 5.123f) < 1e-6f && out.stamp.availableTime == 1.1);
	CHECK(fabsf(out.rayEnd.z - 8.1234f) < 1e-5f);
	CHECK(out.origin.x == 1 && out.direction.z == 1);
	d.mount.offset = YMPME_Vec3(1, 0, 0);
	s.rotation = (pme_rotation){{{0, 0, -1}, {0, 1, 0}, {1, 0, 0}}};
	CHECK(YMPME_Laser_Measure(&d, &s, &stamp, &p, &seed, &out) == PME_OK);
	CHECK(f.ray.origin.z == 2 && f.ray.direction.x == 1);
	f.distance = .01f;
	CHECK(YMPME_Laser_Measure(&d, &s, &stamp, &p, &seed, &out) == PME_OK);
	CHECK(!out.valid && out.hit && out.status == PME_LASER_BLIND && isnan(out.distance));
	CHECK(fabsf(out.rayEnd.x - out.origin.x - .01f) < 1e-6f);
	f.hit = 0;
	CHECK(YMPME_Laser_Measure(&d, &s, &stamp, &p, &seed, &out) == PME_OK);
	CHECK(!out.valid && !out.hit && out.body == SIZE_MAX && out.status == PME_LASER_NO_RETURN &&
		  isnan(out.distance));
	CHECK(fabsf(out.rayEnd.x - out.origin.x - 20) < 1e-6f);
	f.hit = 1;
	f.distance = 20;
	d.resolution = 0;
	CHECK(YMPME_Laser_Measure(&d, &s, &stamp, &p, &seed, &out) == PME_OK && out.valid);
	d.bias = 1;
	CHECK(YMPME_Laser_Measure(&d, &s, &stamp, &p, &seed, &out) == PME_OK &&
		  out.status == PME_LASER_OUT_OF_RANGE && !out.valid);
	d.bias = 0;
	f.distance = 4;
	d.noise = .1f;
	uint32_t a = 42, b = 42;
	pme_laser_sample x, y;
	CHECK(YMPME_Laser_Measure(&d, &s, &stamp, &p, &a, &x) == PME_OK &&
		  YMPME_Laser_Measure(&d, &s, &stamp, &p, &b, &y) == PME_OK);
	CHECK(a == b && x.distance == y.distance && x.rayEnd.x == y.rayEnd.x &&
		  fabsf(x.distance - 4) <= .174f);
	old = out;
	uint32_t saved = seed;
	f.fail = 1;
	CHECK(YMPME_Laser_Measure(&d, &s, &stamp, &p, &seed, &out) == PME_INVALID_ARGUMENT &&
		  !memcmp(&out, &old, sizeof out) && seed == saved);
	f.fail = 0;
	f.distance = NAN;
	CHECK(YMPME_Laser_Measure(&d, &s, &stamp, &p, &seed, &out) == PME_INVALID_ARGUMENT &&
		  !memcmp(&out, &old, sizeof out));
	f.distance = 4;
	d.resolution = -1;
	int calls = f.calls;
	CHECK(YMPME_Laser_Measure(&d, &s, &stamp, &p, &seed, &out) == PME_INVALID_ARGUMENT &&
		  f.calls == calls && !memcmp(&out, &old, sizeof out));
	puts("LASER PASS: one ray, quantization, mounting, blind zone, no return, range/noise, "
		 "transactional failure");
	return 0;
}
