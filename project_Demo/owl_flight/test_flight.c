#include "flight.h"
#include "YMGRE_Camera.h"
#include <stdio.h>
#include <string.h>
static owl_flight s;
#ifdef WRAP_ALLOC
static int tracking;
static unsigned allocations;
void *__real_malloc(size_t);
void *__real_calloc(size_t, size_t);
void *__real_realloc(void *, size_t);
void __real_free(void *);
void *__wrap_malloc(size_t n)
{
	if (tracking)
		allocations++;
	return __real_malloc(n);
}
void *__wrap_calloc(size_t n, size_t m)
{
	if (tracking)
		allocations++;
	return __real_calloc(n, m);
}
void *__wrap_realloc(void *p, size_t n)
{
	if (tracking)
		allocations++;
	return __real_realloc(p, n);
}
void __wrap_free(void *p)
{
	if (tracking)
		allocations++;
	__real_free(p);
}
#endif
#define CHECK(x)                                                                                   \
	do                                                                                             \
	{                                                                                              \
		if (!(x))                                                                                  \
		{                                                                                          \
			fprintf(stderr, "owl flight FAIL line %d: %s\n", __LINE__, #x);                        \
			goto fail;                                                                             \
		}                                                                                          \
	} while (0)
static uint64_t hash(void)
{
	GRE_RenderTarget t = YMGRE_Camera_GetRenderTarget(s.visual.camera);
	const unsigned char *p = (const unsigned char *)t->data;
	uint64_t h = 14695981039346656037ull;
	for (size_t i = 0; i < t->width * t->height * sizeof(*t->data); i += 13)
		h = (h ^ p[i]) * 1099511628211ull;
	return h;
}
int main(void)
{
	CHECK(OwlFlight_Init(&s, NULL));
	CHECK(s.visual.animation.info.frameCount > 1 && s.visual.asset.info.vertexCount == 30015);
	CHECK(OwlFlight_Render(&s, 2, 0, 0));
	uint64_t first = hash();
	pme_aircraft_command c = {0, 0, 0, 0, 1};
	for (int i = 0; i < 600; i++)
		CHECK(OwlFlight_Step(&s, &c, YMPME_Vec3(0, 0, 0)));
	CHECK(s.aircraft.position.z > 15 && s.aircraft.position.y > .3f);
	pme_aircraft base = s.aircraft;
	pme_aircraft level = s.aircraft, climb = s.aircraft, dive = s.aircraft;
	pme_aircraft_command up = {0, 0, 1, 0, 1}, down = {0, 0, -1, 0, 1};
	for (int i = 0; i < 240; i++)
	{
		CHECK(YMPME_Aircraft_Step(&level, &c, YMPME_Vec3(0, 0, 0), 1.f / 240) == PME_OK);
		CHECK(YMPME_Aircraft_Step(&climb, &up, YMPME_Vec3(0, 0, 0), 1.f / 240) == PME_OK);
		CHECK(YMPME_Aircraft_Step(&dive, &down, YMPME_Vec3(0, 0, 0), 1.f / 240) == PME_OK);
	}
	CHECK(climb.position.y > level.position.y + .05f);
	CHECK(dive.position.y < level.position.y - .05f);

	c.right = 1;
	c.climb = 1;
	for (int i = 0; i < 240; i++)
		CHECK(OwlFlight_Step(&s, &c, YMPME_Vec3(0, 0, 0)));
	CHECK(fabsf(s.aircraft.q[1] - base.q[1]) > .02f);
	CHECK(OwlFlight_Render(&s, 2, 0, 0) && Demo_Validate(&s.visual) && hash() != first);
	c = (pme_aircraft_command){0, 0, 0, 0, 0};
	for (int i = 0; i < 360; i++)
		CHECK(OwlFlight_Step(&s, &c, YMPME_Vec3(0, 0, 0)));
	CHECK(s.aircraft.thrust == 0 && s.flapBlend < .0001f);
	CHECK(OwlFlight_Render(&s, 0, 0, 0));
	/* World root transform must agree with PME translation for unchanged pose. */
	gre_fvector4d p = s.visual.object->pointList[120].pos;
	s.aircraft.position.x += 5;
	CHECK(OwlFlight_Render(&s, 0, 0, 0));
	CHECK(fabsf(s.visual.object->pointList[120].pos.x - p.x - 5) < .0001f);
	pme_aircraft before = s.aircraft;
	c.power = 2;
	CHECK(!OwlFlight_Step(&s, &c, YMPME_Vec3(0, 0, 0)));
	CHECK(!memcmp(&before, &s.aircraft, sizeof before));
	CHECK(OwlFlight_Reset(&s));
	CHECK(OwlFlight_Render(&s, 2, 0, 0));
#ifdef WRAP_ALLOC
	tracking = 1;
#endif
	c = (pme_aircraft_command){0, 0, .2f, .4f, 1};
	for (int i = 0; i < 60; i++)
	{
		CHECK(OwlFlight_Step(&s, &c, YMPME_Vec3(1, 0, 0)));
		CHECK(OwlFlight_Render(&s, i % 3, 0, 0));
	}
#ifdef WRAP_ALLOC
	tracking = 0;
	CHECK(allocations == 0);
#endif
	CHECK(Demo_Save(&s.visual, "owl_flight_verified.ppm"));
	puts("OWL_FLIGHT PASS: imported animation, dynamics, controls, glide, world binding, zero "
		 "frame allocations");
	OwlFlight_Destroy(&s);
	return 0;
fail:
#ifdef WRAP_ALLOC
	tracking = 0;
#endif
	OwlFlight_Destroy(&s);
	return 1;
}
