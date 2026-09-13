#include "YMPME_Tactile.h"
#include "../YMPME_RangeInternal.h"
#include <string.h>
static pme_result setup(const pme_tactile_desc *d, const pme_feedback_state *s,
						const pme_sensor_stamp *t, const pme_tactile_frame *out, pme_vec3 *origin,
						pme_rotation *r)
{
	if (!d || !out || !out->cells || !d->width || !d->height || d->height > SIZE_MAX / d->width ||
		!isfinite(d->cellWidth) || d->cellWidth <= 0 || !isfinite(d->cellHeight) ||
		d->cellHeight <= 0 || !isfinite(d->cellWidth * d->cellHeight) ||
		d->cellWidth * d->cellHeight <= 0 || !isfinite(d->planeTolerance) ||
		d->planeTolerance < 0 || !isfinite(d->width * d->cellWidth) ||
		!isfinite(d->height * d->cellHeight) || !YMPME_Range_Pose(s, &d->mount, t, origin, r))
		return PME_INVALID_ARGUMENT;
	if (out->capacity < d->width * d->height || d->width * d->height > SIZE_MAX / sizeof(pme_taxel))
		return PME_CAPACITY_EXCEEDED;
	return PME_OK;
}
static void begin(const pme_tactile_desc *d, pme_tactile_frame *out, pme_vec3 p, pme_rotation r)
{
	out->stamp.valid = 0;
	out->width = d->width;
	out->height = d->height;
	out->accepted = out->rejected = 0;
	out->approximate = 0;
	out->totalForce = YMPME_Vec3(0, 0, 0);
	out->origin = p;
	out->rotation = r;
	memset(out->cells, 0, d->width * d->height * sizeof(*out->cells));
}
static int add(const pme_tactile_desc *d, pme_tactile_frame *out, pme_tactile_contact c)
{
	pme_vec3 p = YMPME_Sensor_ToLocal(out->rotation, YMPME_VecSub(c.point, out->origin)),
			 f = YMPME_Sensor_ToLocal(out->rotation, c.force);
	double x = (double)p.x / d->cellWidth + d->width * .5,
		   y = (double)p.y / d->cellHeight + d->height * .5;
	size_t index;
	pme_taxel *cell;
	out->approximate |= !!c.approximate;
	if (!YMPME_VecFinite(p) || !YMPME_VecFinite(f))
		return 0;
	if (x < 0 || y < 0 || x >= d->width || y >= d->height || fabsf(p.z) > d->planeTolerance ||
		f.z > 0)
	{
		++out->rejected;
		return 1;
	}
	index = (size_t)floor(y) * d->width + (size_t)floor(x);
	cell = &out->cells[index];
	cell->force = YMPME_VecAdd(cell->force, f);
	cell->pressure = fmaxf(0, -cell->force.z) / (d->cellWidth * d->cellHeight);
	++cell->contributions;
	out->totalForce = YMPME_VecAdd(out->totalForce, f);
	++out->accepted;
	return YMPME_VecFinite(cell->force) && isfinite(cell->pressure) &&
		   YMPME_VecFinite(out->totalForce);
}
pme_result YMPME_Tactile_Measure(const pme_tactile_desc *d, const pme_feedback_state *s,
								 const pme_sensor_stamp *stamp, const pme_tactile_contact *contacts,
								 size_t count, pme_tactile_frame *out)
{
	pme_vec3 p;
	pme_rotation r;
	size_t i;
	pme_result status = setup(d, s, stamp, out, &p, &r);
	if (status != PME_OK)
		return status;
	if (count && !contacts)
		return PME_INVALID_ARGUMENT;
	for (i = 0; i < count; ++i)
		if (!YMPME_VecFinite(contacts[i].point) || !YMPME_VecFinite(contacts[i].force))
			return PME_INVALID_ARGUMENT;
	begin(d, out, p, r);
	for (i = 0; i < count; ++i)
		if (!add(d, out, contacts[i]))
			return PME_INVALID_ARGUMENT;
	out->stamp = *stamp;
	return PME_OK;
}
pme_result YMPME_Tactile_FromContacts(const pme_tactile_desc *d, const pme_feedback_state *s,
									  const pme_sensor_stamp *stamp, const pme_world_feedback *f,
									  size_t body, pme_tactile_frame *out)
{
	pme_vec3 p;
	pme_rotation r;
	size_t i;
	pme_result status = setup(d, s, stamp, out, &p, &r);
	if (status != PME_OK)
		return status;
	if (!f || f->time != stamp->time || f->dropped || body >= f->stateCount ||
		!isfinite(f->interval) || f->interval <= 0 || f->count > f->contactCapacity ||
		(f->count && !f->contacts))
		return PME_INVALID_ARGUMENT;
	for (i = 0; i < f->count; ++i)
	{
		const pme_contact_feedback *c = &f->contacts[i];
		if (c->bodyA == body || c->bodyB == body)
		{
			pme_vec3 force =
				YMPME_VecScale(YMPME_VecAdd(c->normalImpulse, c->tangentImpulse), 1 / f->interval);
			if (c->bodyA >= f->stateCount || c->bodyB >= f->stateCount || c->bodyA == c->bodyB ||
				!YMPME_VecFinite(c->point) || !YMPME_VecFinite(force))
				return PME_INVALID_ARGUMENT;
		}
	}
	begin(d, out, p, r);
	for (i = 0; i < f->count; ++i)
	{
		const pme_contact_feedback *c = &f->contacts[i];
		pme_tactile_contact t;
		if (c->bodyA != body && c->bodyB != body)
			continue;
		t.point = c->point;
		t.force = YMPME_VecScale(YMPME_VecAdd(c->normalImpulse, c->tangentImpulse),
								 (c->bodyB == body ? 1 : -1) / f->interval);
		t.approximate = c->approximatePoint;
		if (!add(d, out, t))
			return PME_INVALID_ARGUMENT;
	}
	out->stamp = *stamp;
	return PME_OK;
}
