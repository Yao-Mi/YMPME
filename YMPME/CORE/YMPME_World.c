#include "YMPME_World.h"
#include "../CONFIG/YMPME_PubDefine.h"
#include "../DEBUG/YMPME_Log.h"
#include "YMPME_MathBase.h"
#include "YMPME_Solver.h"
#include <limits.h>
#include <string.h>
static int validConfig(const pme_world_desc *d)
{
	return d && YMPME_VecFinite(d->gravity) && isfinite(d->fixedStep) && d->fixedStep > 0 &&
		   isfinite(1.f / d->fixedStep) && d->maxSubsteps > 0 && d->maxSubsteps <= INT_MAX &&
		   d->solverIterations > 0 && isfinite(d->penetrationSlop) && d->penetrationSlop >= 0 &&
		   isfinite(d->correctionRate) && d->correctionRate > 0 && d->correctionRate <= 1 &&
		   isfinite(d->bounceThreshold) && d->bounceThreshold >= 0;
}
static int validWorld(const pme_world *w)
{
	return w && w->bodies && w->capacity && validConfig(&w->config);
}
pme_world_desc YMPME_World_DefaultDesc(void)
{
	pme_world_desc d;
	d.meshPrecision = 1;
	d.gravity = YMPME_Vec3(0, -9.81f, 0);
	d.fixedStep = YMPME_DEFAULT_STEP;
	d.maxSubsteps = 8;
	d.solverIterations = YMPME_DEFAULT_ITERATIONS;
	d.penetrationSlop = 0.0005f;
	d.correctionRate = 0.8f;
	d.bounceThreshold = 0.5f;
	return d;
}
pme_result YMPME_World_Init(PME_World w, PME_Body storage, size_t capacity,
							const pme_world_desc *desc)
{
	pme_world_desc d = desc ? *desc : YMPME_World_DefaultDesc();
	if (!w || !storage || !capacity || capacity > SIZE_MAX / sizeof(*storage) || !validConfig(&d))
		return YMPME_REPORT(PME_INVALID_ARGUMENT,
							"Invalid world, storage, capacity or world configuration");
	memset(w, 0, sizeof(*w));
	memset(storage, 0, capacity * sizeof(*storage));
	w->config = d;
	{
		unsigned i;
		for (i = 0; i < 32; ++i)
			w->groupMasks[i] = UINT32_MAX;
	}
	w->bodies = storage;
	w->capacity = capacity;
	return PME_OK;
}
PME_Body YMPME_World_AddBody(PME_World w, const pme_body_desc *desc)
{
	size_t i;
	if (!validWorld(w))
	{
		YMPME_LOG(PME_LOG_ERROR, PME_INVALID_ARGUMENT, "AddBody requires a valid world");
		return NULL;
	}
	for (i = 0; i < w->capacity; ++i)
	{
		if (!w->bodies[i].active)
		{
			if (YMPME_Body_Init(&w->bodies[i], desc) != PME_OK)
				return NULL;
			++w->bodyCount;
			return &w->bodies[i];
		}
	}
	YMPME_LOG(PME_LOG_ERROR, PME_CAPACITY_EXCEEDED, "World has no free body slots");
	return NULL;
}
pme_result YMPME_World_RemoveBody(PME_World w, PME_Body body)
{
	size_t i;
	if (!validWorld(w) || !body)
		return YMPME_REPORT(PME_INVALID_ARGUMENT, "RemoveBody requires a valid world and body");
	/* 指针相等比较，避免对不属于该数组的指针做关系比较。 */
	for (i = 0; i < w->capacity; ++i)
	{
		if (&w->bodies[i] == body && body->active)
		{
			memset(body, 0, sizeof(*body));
			--w->bodyCount;
			return PME_OK;
		}
	}
	return YMPME_REPORT(PME_INVALID_ARGUMENT, "Body is inactive or does not belong to this world");
}
static void clearForces(PME_World w)
{
	size_t i;
	for (i = 0; i < w->capacity; ++i)
		w->bodies[i].force = YMPME_Vec3(0, 0, 0);
}
static void stepWorld(PME_World w, float dt, pme_world_feedback *feedback)
{
	size_t i, j;
	unsigned iteration;
	for (i = 0; i < w->capacity; ++i)
	{
		PME_Body b = &w->bodies[i];
		if (!b->active)
			continue;
		b->previousPosition = b->state.position;
		if (b->state.type == PME_STATIC)
			continue;
		if (b->state.type == PME_DYNAMIC)
		{
			pme_vec3 acceleration =
				YMPME_VecAdd(w->config.gravity, YMPME_VecScale(b->force, b->inverseMass));
			b->state.velocity =
				YMPME_VecScale(YMPME_VecAdd(b->state.velocity, YMPME_VecScale(acceleration, dt)),
							   1.0f / (1.0f + b->state.linearDamping * dt));
		}
		b->state.position = YMPME_VecAdd(b->state.position, YMPME_VecScale(b->state.velocity, dt));
	}
	/* 小世界使用 O(iterations * capacity²) 遍历，无接触缓存溢出和帧内分配。 */
	for (iteration = 0; iteration < w->config.solverIterations; ++iteration)
	{
		for (i = 0; i < w->capacity; ++i)
		{
			PME_Body a = &w->bodies[i];
			if (!a->active)
				continue;
			for (j = i + 1; j < w->capacity; ++j)
			{
				PME_Body b = &w->bodies[j];
				pme_contact c;
				if (a->inverseMass + b->inverseMass <= 0 || a->state.sensor || b->state.sensor)
					continue;
				if (YMPME_World_Collide(w, a, b, &c))
				{
					pme_contact_feedback event = {0};
					if (feedback)
					{
						event.bodyA = i;
						event.bodyB = j;
						event.iteration = iteration;
						event.normal = c.normal;
						event.approximatePoint = 1;
						pme_aabb aa, bb;
						YMPME_Body_GetAABB(a, &aa);
						YMPME_Body_GetAABB(b, &bb);
						/* 用世界AABB重叠区域中心估计接触位置，不代表精确接触流形。 */
						event.point = YMPME_VecScale(
							YMPME_VecAdd(
								YMPME_Vec3(fmaxf(aa.min.x, bb.min.x), fmaxf(aa.min.y, bb.min.y),
										   fmaxf(aa.min.z, bb.min.z)),
								YMPME_Vec3(fminf(aa.max.x, bb.max.x), fminf(aa.max.y, bb.max.y),
										   fminf(aa.max.z, bb.max.z))),
							.5f);
					}
					YMPME_Solver_ResolveFeedback(a, b, &c, &w->config, feedback ? &event : NULL);
					if (feedback && (YMPME_VecDot(event.normalImpulse, event.normalImpulse) > 0 ||
									 YMPME_VecDot(event.tangentImpulse, event.tangentImpulse) > 0))
					{
						if (feedback->count < feedback->contactCapacity)
							feedback->contacts[feedback->count++] = event;
						else
							++feedback->dropped;
					}
				}
			}
		}
	}
	++w->stepCount;
}
pme_result YMPME_World_Step(PME_World w, float dt)
{
	if (!validWorld(w) || !isfinite(dt) || dt <= 0 || !isfinite(1.f / dt))
		return YMPME_REPORT(PME_INVALID_ARGUMENT,
							"Step requires a valid world and finite positive timestep");
	memset(&w->collisionStats, 0, sizeof(w->collisionStats));
	stepWorld(w, dt, NULL);
	clearForces(w);
	return PME_OK;
}
int YMPME_World_Advance(PME_World w, float frameTime)
{
	unsigned steps = 0;
	double dt;
	if (!validWorld(w) || !isfinite(frameTime) || frameTime < 0)
		return YMPME_REPORT(PME_INVALID_ARGUMENT,
							"Advance requires a valid world and finite nonnegative frame time");
	memset(&w->collisionStats, 0, sizeof(w->collisionStats));
	dt = w->config.fixedStep;
	w->accumulator += frameTime;
	while (w->accumulator >= dt && steps < w->config.maxSubsteps)
	{
		stepWorld(w, w->config.fixedStep, NULL);
		w->accumulator -= dt;
		++steps;
	}
	if (steps)
		clearForces(w);
	if (w->accumulator >= dt)
	{
		double remainder = fmod(w->accumulator, dt);
		w->droppedTime += w->accumulator - remainder;
		w->accumulator = remainder;
	}
	return (int)steps;
}
float YMPME_World_InterpolationAlpha(const pme_world *w)
{
	if (!validWorld(w))
		return 0;
	return YMPME_Clamp((float)(w->accumulator / w->config.fixedStep), 0, 1);
}
pme_result YMPME_World_Destroy(PME_World w)
{
	if (!validWorld(w) || w->ownsMemory)
		return YMPME_REPORT(PME_INVALID_ARGUMENT,
							"Destroy requires a valid caller-owned world; use Free "
							"for allocated worlds");
	memset(w->bodies, 0, w->capacity * sizeof(*w->bodies));
	memset(w, 0, sizeof(*w));
	return PME_OK;
}

pme_result YMPME_World_SetGroupCollision(PME_World w, uint32_t a, uint32_t b, int enabled)
{
	if (!validWorld(w) || a >= 32 || b >= 32)
		return YMPME_REPORT(PME_INVALID_ARGUMENT,
							"Collision group indices must be in 0..31 and world must be valid");
	if (enabled)
	{
		w->groupMasks[a] |= UINT32_C(1) << b;
		w->groupMasks[b] |= UINT32_C(1) << a;
	}
	else
	{
		w->groupMasks[a] &= ~(UINT32_C(1) << b);
		w->groupMasks[b] &= ~(UINT32_C(1) << a);
	}
	return PME_OK;
}
int YMPME_World_Collide(PME_World w, const pme_body *a, const pme_body *b, pme_contact *contact)
{
	pme_collision_options options;
	if (!validWorld(w))
		return 0;
	options.meshPrecision = w->config.meshPrecision;
	options.groupMasks = w->groupMasks;
	options.stats = &w->collisionStats;
	return YMPME_CollideWithOptions(a, b, &options, contact);
}

pme_result YMPME_World_StepWithFeedback(PME_World w, float dt, pme_world_feedback *f)
{
	if (!validWorld(w) || !isfinite(dt) || dt <= 0 || !isfinite(1.f / dt) || !f ||
		!isfinite(f->time) || f->time < 0 || !f->states || f->stateCapacity < w->capacity ||
		(f->contactCapacity && !f->contacts))
		return YMPME_REPORT(PME_INVALID_ARGUMENT,
							"Invalid feedback buffers, capacities, time, world or timestep");
	f->count = f->dropped = 0;
	f->stateCount = w->capacity;
	f->interval = dt;
	for (size_t i = 0; i < w->capacity; ++i)
	{
		pme_feedback_state *s = &f->states[i];
		memset(s, 0, sizeof(*s));
		s->velocity = w->bodies[i].state.velocity;
	}
	memset(&w->collisionStats, 0, sizeof(w->collisionStats));
	stepWorld(w, dt, f);
	for (size_t i = 0; i < w->capacity; ++i)
	{
		pme_feedback_state *s = &f->states[i];
		const pme_body *b = &w->bodies[i];
		if (!b->active)
		{
			memset(s, 0, sizeof(*s));
			continue;
		}
		pme_vec3 before = s->velocity;
		s->time = f->time;
		s->interval = dt;
		s->position = b->state.position;
		s->rotation =
			b->state.shape == PME_CONVEX_MESH ? b->state.rotation : YMPME_Rotation_Identity();
		s->velocity = b->state.type == PME_STATIC ? YMPME_Vec3(0, 0, 0) : b->state.velocity;
		s->acceleration = b->state.type == PME_STATIC
							  ? YMPME_Vec3(0, 0, 0)
							  : YMPME_VecScale(YMPME_VecSub(s->velocity, before), 1 / dt);
		s->valid = PME_FEEDBACK_POSE | PME_FEEDBACK_VELOCITY | PME_FEEDBACK_ACCELERATION;
		if (b->state.type == PME_STATIC)
			s->valid |= PME_FEEDBACK_ANGULAR_VELOCITY | PME_FEEDBACK_ANGULAR_ACCELERATION;
	}
	clearForces(w);
	return PME_OK;
}
