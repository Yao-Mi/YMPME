#include "YMPME_TactileWindow.h"
#include <string.h>
pme_result YMPME_TactileWindow_Begin(pme_tactile_window *w, pme_taxel *cells, size_t capacity)
{
	if (!w || !cells || !capacity || capacity > SIZE_MAX / sizeof(*cells))
		return PME_INVALID_ARGUMENT;
	*w = (pme_tactile_window){0};
	w->integrals = cells;
	w->capacity = capacity;
	return PME_OK;
}
pme_result YMPME_TactileWindow_Add(pme_tactile_window *w, const pme_tactile_frame *f, float dt)
{
	size_t i, n;
	double duration;
	pme_taxel zero = {0};
	if (!w || !w->integrals || !f || !f->cells || !f->stamp.valid || !isfinite(f->stamp.time) ||
		!isfinite(f->stamp.availableTime) || f->stamp.availableTime < f->stamp.time ||
		!isfinite(dt) || dt <= 0 || f->stamp.time + 1e-7 < dt || !f->width || !f->height ||
		f->height > SIZE_MAX / f->width || !YMPME_VecFinite(f->origin) ||
		!YMPME_Rotation_IsValid(f->rotation))
		return PME_INVALID_ARGUMENT;
	n = f->width * f->height;
	if (f->capacity < n || w->capacity < n)
		return PME_CAPACITY_EXCEEDED;
	if (w->active &&
		(f->width != w->width || f->height != w->height || f->stamp.time <= w->endTime ||
		 fabs(f->stamp.time - dt - w->endTime) > fmax(1e-7, dt * 1e-5)))
		return PME_INVALID_ARGUMENT;
	duration = (w->active ? w->duration : 0) + dt;
	if (!isfinite(duration) || duration <= 0 ||
		f->accepted > SIZE_MAX - (w->active ? w->accepted : 0) ||
		f->rejected > SIZE_MAX - (w->active ? w->rejected : 0))
		return PME_INVALID_ARGUMENT;
	/* Check all sums before the first buffer write. */
	for (i = 0; i < n; ++i)
	{
		const pme_taxel *old = w->active ? &w->integrals[i] : &zero, *add = &f->cells[i];
		if (!YMPME_VecFinite(add->force) || !isfinite(add->pressure) || add->pressure < 0 ||
			!YMPME_VecFinite(YMPME_VecAdd(old->force, YMPME_VecScale(add->force, dt))) ||
			!isfinite(old->pressure + add->pressure * dt) ||
			add->contributions > SIZE_MAX - old->contributions)
			return PME_INVALID_ARGUMENT;
	}
	if (!w->active)
	{
		memset(w->integrals, 0, n * sizeof(*w->integrals));
		w->startTime = f->stamp.time - dt;
		w->accepted = w->rejected = 0;
		w->approximate = 0;
		w->availableTime = 0;
	}
	for (i = 0; i < n; ++i)
	{
		w->integrals[i].force =
			YMPME_VecAdd(w->integrals[i].force, YMPME_VecScale(f->cells[i].force, dt));
		w->integrals[i].pressure += f->cells[i].pressure * dt;
		w->integrals[i].contributions += f->cells[i].contributions;
	}
	w->width = f->width;
	w->height = f->height;
	w->endTime = f->stamp.time;
	w->duration = duration;
	w->availableTime = fmax(w->availableTime, f->stamp.availableTime);
	w->accepted += f->accepted;
	w->rejected += f->rejected;
	w->origin = f->origin;
	w->rotation = f->rotation;
	w->active = 1;
	w->approximate |= f->approximate;
	return PME_OK;
}
pme_result YMPME_TactileWindow_Read(const pme_tactile_window *w, pme_tactile_frame *out)
{
	size_t i, n;
	float scale;
	pme_tactile_frame frame;
	if (!w || !w->active || !w->integrals || !out || !out->cells || !isfinite(w->duration) ||
		w->duration <= 0 || !w->width || !w->height || w->height > SIZE_MAX / w->width)
		return PME_INVALID_ARGUMENT;
	n = w->width * w->height;
	if (out->capacity < n || w->capacity < n)
		return PME_CAPACITY_EXCEEDED;
	scale = (float)(1 / w->duration);
	if (!isfinite(scale))
		return PME_INVALID_ARGUMENT;
	frame = *out;
	frame.totalForce = YMPME_Vec3(0, 0, 0);
	for (i = 0; i < n; ++i)
	{
		pme_vec3 force = YMPME_VecScale(w->integrals[i].force, scale);
		float pressure = w->integrals[i].pressure * scale;
		if (!YMPME_VecFinite(force) || !isfinite(pressure) || pressure < 0)
			return PME_INVALID_ARGUMENT;
		frame.totalForce = YMPME_VecAdd(frame.totalForce, force);
	}
	if (!YMPME_VecFinite(frame.totalForce))
		return PME_INVALID_ARGUMENT;
	for (i = 0; i < n; ++i)
	{
		frame.cells[i] = w->integrals[i];
		frame.cells[i].force = YMPME_VecScale(frame.cells[i].force, scale);
		frame.cells[i].pressure *= scale;
	}
	frame.stamp = (pme_sensor_stamp){w->endTime, w->availableTime, 0, 1};
	frame.origin = w->origin;
	frame.rotation = w->rotation;
	frame.width = w->width;
	frame.height = w->height;
	frame.accepted = w->accepted;
	frame.rejected = w->rejected;
	frame.approximate = w->approximate;
	*out = frame;
	return PME_OK;
}
