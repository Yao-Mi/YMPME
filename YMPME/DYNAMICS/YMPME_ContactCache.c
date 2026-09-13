#include "YMPME_ContactCache.h"
#include <limits.h>
#include <string.h>
#define add YMPME_VecAdd
#define sub YMPME_VecSub
#define mul YMPME_VecScale
#define dot YMPME_VecDot
static float distance2(pme_vec3 a, pme_vec3 b)
{
	pme_vec3 d = sub(a, b);
	return dot(d, d);
}
pme_result YMPME_Dynamics_CacheInit(pme_contact_cache *c, pme_contact_cache_entry *e,
									size_t capacity)
{
	if (!c || (capacity && !e) || capacity > SIZE_MAX / sizeof(*e))
		return PME_INVALID_ARGUMENT;
	pme_contact_cache v = {0};
	v.entries = e;
	v.capacity = capacity;
	v.anchorTolerance = .02f;
	v.normalCosine = .95f;
	*c = v;
	return PME_OK;
}
void YMPME_Dynamics_CacheClear(pme_contact_cache *c)
{
	if (!c)
		return;
	c->count = c->hits = 0;
	c->interval = 0;
	c->time = 0;
	c->bodies = NULL;
	c->colliders = NULL;
	c->bodyCount = c->colliderCount = 0;
}
static size_t low(size_t a, size_t b)
{
	return a < b ? a : b;
}
static size_t high(size_t a, size_t b)
{
	return a > b ? a : b;
}
static int pairLess(size_t a, size_t b, size_t c, size_t d)
{
	size_t x = low(a, b), y = low(c, d);
	return x < y || (x == y && high(a, b) < high(c, d));
}
int PME_ContactCache_Valid(const pme_contact_cache *c)
{
	if ((c->capacity && !c->entries) || c->capacity > SIZE_MAX / sizeof(*c->entries) ||
		c->count > c->capacity || !isfinite(c->anchorTolerance) || c->anchorTolerance <= 0 ||
		!isfinite(c->anchorTolerance * c->anchorTolerance) || !isfinite(c->normalCosine) ||
		c->normalCosine < 0 || c->normalCosine > 1 || !isfinite(c->time) ||
		!isfinite(c->interval) || c->interval < 0 || (c->count && c->interval <= 0))
		return 0;
	for (size_t i = 0; i < c->count; ++i)
	{
		const pme_contact_cache_entry *e = &c->entries[i];
		if (!YMPME_VecFinite(e->localPointA) || !YMPME_VecFinite(e->localPointB) ||
			!YMPME_VecFinite(e->normal) || !YMPME_VecFinite(e->tangentImpulse) ||
			!isfinite(e->normalImpulse) || e->normalImpulse < 0 ||
			fabsf(dot(e->normal, e->normal) - 1) > 1e-3f)
			return 0;
		if (i && pairLess(e->colliderA, e->colliderB, c->entries[i - 1].colliderA,
						  c->entries[i - 1].colliderB))
			return 0;
	}
	return 1;
}
static void seed(pme_joint_row *r, pme_rigid_body *a, pme_rigid_body *b, float impulse)
{
	r->impulse = impulse;
	a->velocity = add(a->velocity, mul(r->responseLinearA, impulse));
	a->angularVelocity = add(a->angularVelocity, mul(r->responseAngularA, impulse));
	b->velocity = add(b->velocity, mul(r->responseLinearB, impulse));
	b->angularVelocity = add(b->angularVelocity, mul(r->responseAngularB, impulse));
}
void PME_ContactCache_WarmStart(pme_dynamics *w, float dt)
{
	pme_contact_cache *cache = w->cache;
	cache->hits = 0;
	if (cache->bodies != w->bodies || cache->colliders != w->colliders ||
		cache->bodyCount != w->bodyCount || cache->colliderCount != w->colliderCount ||
		cache->time != w->time || cache->interval <= 0)
		return;
	float ratio = dt / cache->interval;
	if (ratio < .5f || ratio > 2)
		return; /* large timestep changes cold-start automatically */
	for (size_t i = 0; i < cache->count; ++i)
		cache->entries[i].used = 0;
	size_t cursor = 0;
	float tolerance2 = cache->anchorTolerance * cache->anchorTolerance;
	for (size_t i = 0; i < w->contactCount; ++i)
	{
		pme_dynamic_contact *c = &w->contacts[i];
		/* Both arrays follow broad-phase pair order. Merge pairs, then compare only
		 * the (at most eight) cached points of this pair; never scan all contacts. */
		while (cursor < cache->count &&
			   pairLess(cache->entries[cursor].colliderA, cache->entries[cursor].colliderB,
						c->colliderA, c->colliderB))
			++cursor;
		size_t best = SIZE_MAX;
		float score = 2 * tolerance2;
		for (size_t j = cursor; j < cache->count; ++j)
		{
			pme_contact_cache_entry *e = &cache->entries[j];
			if (pairLess(c->colliderA, c->colliderB, e->colliderA, e->colliderB))
				break;
			if (e->used || e->colliderA != c->colliderA || e->colliderB != c->colliderB ||
				e->bodyA != c->bodyA || e->bodyB != c->bodyB ||
				dot(e->normal, c->normal) < cache->normalCosine)
				continue;
			float da = distance2(e->localPointA, c->localPointA),
				  db = distance2(e->localPointB, c->localPointB);
			if (da <= tolerance2 && db <= tolerance2 && da + db <= score)
			{
				best = j;
				score = da + db;
			}
		}
		if (best == SIZE_MAX)
			continue;
		pme_contact_cache_entry *e = &cache->entries[best];
		float normal = e->normalImpulse * ratio;
		pme_vec3 tangent = mul(e->tangentImpulse, ratio);
		float u = dot(tangent, c->rows[1].linearB), v = dot(tangent, c->rows[2].linearB);
		float magnitude = hypotf(u, v), limit = c->friction * normal;
		if (!isfinite(normal) || !isfinite(u) || !isfinite(v) || !isfinite(magnitude) ||
			!isfinite(limit))
			continue;
		if (magnitude > limit && magnitude > 0)
		{
			u *= limit / magnitude;
			v *= limit / magnitude;
		}
		pme_rigid_body *a = &w->bodies[c->bodyA], *b = &w->bodies[c->bodyB];
		seed(&c->rows[0], a, b, normal);
		seed(&c->rows[1], a, b, u);
		seed(&c->rows[2], a, b, v);
		e->used = 1;
		c->persistence = e->persistence == UINT_MAX ? UINT_MAX : e->persistence + 1;
		++cache->hits;
	}
}
void PME_ContactCache_Store(pme_dynamics *w, float dt, double time)
{
	pme_contact_cache *cache = w->cache;
	for (size_t i = 0; i < w->contactCount; ++i)
	{
		const pme_dynamic_contact *c = &w->contacts[i];
		cache->entries[i] = (pme_contact_cache_entry){
			c->colliderA,	  c->colliderB,	  c->bodyA,	 c->bodyB,
			c->localPointA,	  c->localPointB, c->normal, c->tangentImpulse,
			c->normalImpulse, c->persistence, 0};
	}
	cache->count = w->contactCount;
	cache->interval = dt;
	cache->time = time;
	cache->bodies = w->bodies;
	cache->colliders = w->colliders;
	cache->bodyCount = w->bodyCount;
	cache->colliderCount = w->colliderCount;
}
