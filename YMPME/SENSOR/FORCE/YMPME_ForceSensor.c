#include "YMPME_ForceSensor.h"
pme_result YMPME_ForceSensor_Contacts(const pme_world_feedback *f, size_t body, pme_vec3 origin,
									  pme_force_sample *out)
{
	if (!f || !out || !f->states || body >= f->stateCount || f->stateCount > f->stateCapacity ||
		!f->states[body].valid || f->dropped || f->count > f->contactCapacity ||
		(f->count && !f->contacts) || !isfinite(f->interval) || f->interval <= 0 ||
		!isfinite(f->time) || !YMPME_VecFinite(origin))
		return PME_INVALID_ARGUMENT;
	pme_force_sample result = {0};
	result.stamp = (pme_sensor_stamp){f->time, f->time, 0, 1};
	for (size_t i = 0; i < f->count; ++i)
	{
		const pme_contact_feedback *e = &f->contacts[i];
		if (e->bodyA != body && e->bodyB != body)
			continue;
		pme_vec3 impulse = YMPME_VecScale(YMPME_VecAdd(e->normalImpulse, e->tangentImpulse),
										  e->bodyB == body ? 1 : -1);
		result.wrench.force = YMPME_VecAdd(result.wrench.force, impulse);
		result.wrench.torque = YMPME_VecAdd(
			result.wrench.torque, YMPME_VecCross(YMPME_VecSub(e->point, origin), impulse));
		result.approximateTorque |= e->approximatePoint;
	}
	result.wrench.force = YMPME_VecScale(result.wrench.force, 1 / f->interval);
	result.wrench.torque = YMPME_VecScale(result.wrench.torque, 1 / f->interval);
	if (!YMPME_VecFinite(result.wrench.force) || !YMPME_VecFinite(result.wrench.torque))
		return PME_INVALID_ARGUMENT;
	*out = result;
	return PME_OK;
}
pme_result YMPME_ForceSensor_Measure(const pme_wrench *w, const pme_feedback_state *s,
									 const pme_sensor_mount *m, const pme_sensor_stamp *stamp,
									 pme_force_sample *out)
{
	if (!w || !s || !m || !stamp || !out || !stamp->valid || stamp->time != s->time ||
		!isfinite(stamp->time) || !isfinite(stamp->availableTime) ||
		stamp->availableTime < stamp->time || !(s->valid & PME_FEEDBACK_POSE) ||
		!YMPME_Rotation_IsValid(s->rotation) || !YMPME_Rotation_IsValid(m->rotation) ||
		!YMPME_VecFinite(m->offset) || !YMPME_VecFinite(w->force) || !YMPME_VecFinite(w->torque))
		return PME_INVALID_ARGUMENT;
	pme_rotation rot = YMPME_Sensor_WorldRotation(s->rotation, m->rotation);
	pme_vec3 r = YMPME_Rotation_Apply(s->rotation, m->offset);
	pme_force_sample sample = {0};
	sample.stamp = *stamp;
	sample.wrench.force = YMPME_Sensor_ToLocal(rot, w->force);
	sample.wrench.torque =
		YMPME_Sensor_ToLocal(rot, YMPME_VecSub(w->torque, YMPME_VecCross(r, w->force)));
	if (!YMPME_VecFinite(sample.wrench.force) || !YMPME_VecFinite(sample.wrench.torque))
		return PME_INVALID_ARGUMENT;
	*out = sample;
	return PME_OK;
}

pme_result YMPME_ForceWindow_Begin(pme_force_window *w, size_t body, pme_vec3 origin)
{
	if (!w || !YMPME_VecFinite(origin))
		return PME_INVALID_ARGUMENT;
	*w = (pme_force_window){0};
	w->body = body;
	w->origin = origin;
	return PME_OK;
}
pme_result YMPME_ForceWindow_Add(pme_force_window *w, const pme_world_feedback *f)
{
	if (!w || !f)
		return PME_INVALID_ARGUMENT;
	if (w->active && (f->time <= w->endTime ||
					  fabs(f->time - f->interval - w->endTime) > fmax(1e-7, f->interval * 1e-5)))
		return PME_INVALID_ARGUMENT;
	pme_force_sample sample;
	if (YMPME_ForceSensor_Contacts(f, w->body, w->origin, &sample) != PME_OK)
		return PME_INVALID_ARGUMENT;
	pme_force_window result = *w;
	result.impulse.force =
		YMPME_VecAdd(result.impulse.force, YMPME_VecScale(sample.wrench.force, f->interval));
	result.impulse.torque =
		YMPME_VecAdd(result.impulse.torque, YMPME_VecScale(sample.wrench.torque, f->interval));
	if (!result.active)
		result.startTime = f->time - f->interval;
	result.endTime = f->time;
	result.duration += f->interval;
	result.active = 1;
	result.approximateTorque |= sample.approximateTorque;
	if (!YMPME_VecFinite(result.impulse.force) || !YMPME_VecFinite(result.impulse.torque) ||
		!isfinite(result.duration))
		return PME_INVALID_ARGUMENT;
	*w = result;
	return PME_OK;
}
pme_result YMPME_ForceWindow_Read(const pme_force_window *w, pme_force_sample *out)
{
	if (!w || !out || !w->active || !isfinite(w->duration) || w->duration <= 0)
		return PME_INVALID_ARGUMENT;
	pme_force_sample s = {0};
	s.stamp = (pme_sensor_stamp){w->endTime, w->endTime, 0, 1};
	s.approximateTorque = w->approximateTorque;
	s.wrench.force = YMPME_VecScale(w->impulse.force, (float)(1 / w->duration));
	s.wrench.torque = YMPME_VecScale(w->impulse.torque, (float)(1 / w->duration));
	if (!YMPME_VecFinite(s.wrench.force) || !YMPME_VecFinite(s.wrench.torque))
		return PME_INVALID_ARGUMENT;
	*out = s;
	return PME_OK;
}
