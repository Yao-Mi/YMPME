#include "YMPME_Vehicle.h"
#include "../SUSPENSION/YMPME_Suspension.h"
#include <string.h>
static pme_vec3 rotate(pme_vec3 p, float yaw, float pitch, float roll)
{
	float x = p.x * cosf(roll) - p.y * sinf(roll), y = p.x * sinf(roll) + p.y * cosf(roll), z;
	z = -y * sinf(pitch) + p.z * cosf(pitch);
	y = y * cosf(pitch) + p.z * sinf(pitch);
	return YMPME_Vec3(x * cosf(yaw) + z * sinf(yaw), y, -x * sinf(yaw) + z * cosf(yaw));
}
pme_rotation YMPME_Vehicle_Rotation(const pme_vehicle *v)
{
	pme_rotation r;
	r.axis[0] = rotate(YMPME_Vec3(1, 0, 0), v->drive.yaw, v->pitch, v->roll);
	r.axis[1] = rotate(YMPME_Vec3(0, 1, 0), v->drive.yaw, v->pitch, v->roll);
	r.axis[2] = rotate(YMPME_Vec3(0, 0, 1), v->drive.yaw, v->pitch, v->roll);
	return r;
}
static void supportInit(pme_wheel_support *s, float x, float z, float radius, float rest, float travel)
{
	memset(s, 0, sizeof(*s));
	s->x = x;
	s->z = z;
	s->radius = radius;
	s->rest = rest;
	s->travel = travel;
}
void YMPME_Vehicle_UpdateSupports(pme_vehicle *v, const pme_terrain *terrain)
{
	int i;
	pme_rotation r = YMPME_Vehicle_Rotation(v);
	v->contactCount = 0;
	for (i = 0; i < v->supportCount; ++i)
	{
		pme_wheel_support *s = &v->support[i];
		pme_vec3 local = YMPME_Vec3(s->x, 0, s->z), offset;
		float h = 0, distance;
		if (v->kind == PME_VEH_TRICYCLE && i == 2)
		{
			local.x = -v->drive.casterTrail * sinf(v->drive.casterAngle);
			local.z = -0.39f - v->drive.casterTrail * cosf(v->drive.casterAngle);
		}
		offset = YMPME_Rotation_Apply(r, local);
		s->mount = YMPME_VecAdd(YMPME_Vec3(v->drive.x, v->y, v->drive.z), offset);
		if (!YMPME_Terrain_Sample(terrain, s->mount.x, s->mount.z, &h, NULL))
			h = 0;
		s->groundHeight = h;
		distance = s->mount.y - h - s->radius;
		s->length = YMPME_Clamp(distance, s->rest - s->travel, s->rest);
		s->compression = YMPME_Clamp(s->rest - distance, 0, s->travel);
		s->grounded = distance <= s->rest;
		s->position = s->mount;
		s->position.y = s->mount.y - s->length;
		if (s->grounded)
			++v->contactCount;
	}
}
void YMPME_Vehicle_Init(pme_vehicle *v, pme_vehicle_kind kind, float x, float z, const pme_terrain *terrain)
{
	int i;
	float h = 0;
	memset(v, 0, sizeof(*v));
	YMPME_Drive_Reset(&v->drive);
	v->kind = kind;
	v->drive.x = x;
	v->drive.z = z;
	if (kind == PME_VEH_FOUR_WHEEL)
	{
		v->mass = 120;
		v->stiffness = 4800;
		v->damping = 450;
		v->inertiaPitch = 40;
		v->inertiaRoll = 21;
		v->supportCount = 4;
		for (i = 0; i < 4; ++i)
			supportInit(&v->support[i], i % 2 ? 0.6f : -0.6f, i < 2 ? 0.86f : -0.86f, 0.30f, 0.46f, 0.30f);
	}
	else if (kind == PME_VEH_TRACKED)
	{
		v->mass = 140;
		v->stiffness = 10500;
		v->damping = 650;
		v->inertiaPitch = 55;
		v->inertiaRoll = 30;
		v->supportCount = 6;
		v->drive.track = 1.3f;
		v->drive.radius = 0.24f;
		for (i = 0; i < 6; ++i)
			supportInit(&v->support[i], i < 3 ? -0.65f : 0.65f, (i % 3 - 1) * 0.70f, 0.24f, 0.22f, 0.13f);
	}
	else
	{
		v->drive.casterZ = -.39f;
		v->mass = 28;
		v->stiffness = 3500;
		v->damping = 140;
		v->inertiaPitch = 4.5f;
		v->inertiaRoll = 3.5f;
		v->supportCount = 3;
		supportInit(&v->support[0], -0.52f, 0.18f, 0.26f, 0.14f, 0.09f);
		supportInit(&v->support[1], 0.52f, 0.18f, 0.26f, 0.14f, 0.09f);
		supportInit(&v->support[2], 0, -0.53f, 0.13f, 0.27f, 0.13f);
	}
	YMPME_Terrain_Sample(terrain, x, z, &h, NULL);
	v->y = h + v->support[0].radius + v->support[0].rest - v->mass * 9.81f / (v->supportCount * v->stiffness);
	YMPME_Vehicle_UpdateSupports(v, terrain);
}
int YMPME_Vehicle_Step(pme_vehicle *v, const pme_terrain *terrain, float throttle, float steer, int brake,
					   float dt)
{
	if (!v || !terrain)
		return 0;
	float oldX = v->drive.x, oldZ = v->drive.z, oldYaw = v->drive.yaw;
	float total = -v->mass * 9.81f, torquePitch = 0, torqueRoll = 0;
	int i;
	if (!isfinite(dt) || dt <= 0 || dt > 1.0f / 60 || !isfinite(throttle) || !isfinite(steer))
		return 0;
	throttle = YMPME_Clamp(throttle, -1, 1);
	steer = YMPME_Clamp(steer, -1, 1);
	if (v->kind == PME_VEH_FOUR_WHEEL)
	{
		float target = brake ? 0 : throttle * 2.8f, omega, angle, distance, sinc;
		v->speed += YMPME_Clamp(target - v->speed, -(brake ? 10 : 2.5f) * dt, (brake ? 10 : 2.5f) * dt);
		v->steering += YMPME_Clamp(steer * 0.48f - v->steering, -1.7f * dt, 1.7f * dt);
		omega = v->speed * tanf(v->steering) / 1.72f;
		angle = omega * dt;
		sinc = fabsf(angle) < 1e-5f ? 1 : sinf(angle * 0.5f) / (angle * 0.5f);
		distance = v->speed * dt * sinc;
		v->drive.x += distance * sinf(v->drive.yaw + angle * 0.5f);
		v->drive.z += distance * cosf(v->drive.yaw + angle * 0.5f);
		v->drive.yaw = remainderf(v->drive.yaw + angle, 6.28318530718f);
		for (i = 0; i < 4; ++i)
		{
			v->wheelSteer[i] = 0;
			if (i < 2 && fabsf(v->steering) > 1e-5f)
			{
				float radius = 1.72f / tanf(v->steering);
				v->wheelSteer[i] = atanf(1.72f / (radius - v->support[i].x));
			}
			v->wheelAngle[i] = remainderf(v->wheelAngle[i] + (v->speed - omega * v->support[i].x) * dt / 0.3f,
										  6.28318530718f);
		}
	}
	else
	{
		float left = throttle + steer, right = throttle - steer,
			  scale = fmaxf(1, fmaxf(fabsf(left), fabsf(right)));
		if (brake)
			v->drive.left = v->drive.right = 0;
		if (YMPME_Drive_Step(&v->drive, brake ? 0 : 2.0f * left / scale, brake ? 0 : 2.0f * right / scale,
							 dt) != PME_OK)
			return 0;
		v->speed = (v->drive.left + v->drive.right) * 0.5f;
		if (v->kind == PME_VEH_TRACKED)
		{
			v->trackDistance[0] = remainderf(v->trackDistance[0] + v->drive.left * dt, PME_TRACK_LOOP_LENGTH);
			v->trackDistance[1] =
				remainderf(v->trackDistance[1] + v->drive.right * dt, PME_TRACK_LOOP_LENGTH);
		}
		for (i = 0; i < v->supportCount; ++i)
		{
			v->wheelAngle[i] =
				v->kind == PME_VEH_TRACKED
					? (i < 3 ? v->drive.leftAngle : v->drive.rightAngle)
					: (i == 0 ? v->drive.leftAngle : (i == 1 ? v->drive.rightAngle : v->drive.casterSpin));
			v->wheelSteer[i] = (v->kind == PME_VEH_TRICYCLE && i == 2) ? v->drive.casterAngle : 0;
		}
	}
	if (v->worldLimit > 0)
	{
		v->drive.x = YMPME_Clamp(v->drive.x, -v->worldLimit, v->worldLimit);
		v->drive.z = YMPME_Clamp(v->drive.z, -v->worldLimit, v->worldLimit);
	}
	v->vx = (v->drive.x - oldX) / dt;
	v->vz = (v->drive.z - oldZ) / dt;
	v->yawRate = remainderf(v->drive.yaw - oldYaw, 6.28318530718f) / dt;
	YMPME_Vehicle_UpdateSupports(v, terrain);
	for (i = 0; i < v->supportCount; ++i)
	{
		pme_wheel_support *s = &v->support[i];
		pme_vec3 n, offset = YMPME_VecSub(s->mount, YMPME_Vec3(v->drive.x, v->y, v->drive.z));
		float ground = 0, groundRate = 0, mountRate, force = 0, leverX, leverZ;
		float distance = s->mount.y - s->groundHeight - s->radius;
		leverX = offset.x * cosf(v->drive.yaw) - offset.z * sinf(v->drive.yaw);
		leverZ = offset.x * sinf(v->drive.yaw) + offset.z * cosf(v->drive.yaw);
		if (YMPME_Terrain_Sample(terrain, s->mount.x, s->mount.z, &ground, &n))
			groundRate =
				(-n.x * (v->vx + v->yawRate * offset.z) - n.z * (v->vz - v->yawRate * offset.x)) / n.y;
		mountRate = v->vy + leverZ * v->pitchRate + leverX * v->rollRate;
		if (s->grounded)
		{
			pme_suspension_desc spring = {v->stiffness, v->damping, s->rest,
										  s->travel,	8,			v->mass * 9.81f * 3};
			if (YMPME_Suspension_Force(&spring, distance, groundRate - mountRate, &force) != PME_OK)
				return 0;
		}
		s->force = force;
		total += force;
		torquePitch += leverZ * force;
		torqueRoll += leverX * force;
	}
	v->vy += total / v->mass * dt;
	v->y += v->vy * dt;
	v->pitchRate += (torquePitch / v->inertiaPitch - 2 * v->pitchRate) * dt;
	v->rollRate += (torqueRoll / v->inertiaRoll - 2 * v->rollRate) * dt;
	v->pitch += v->pitchRate * dt;
	v->roll += v->rollRate * dt;
	/* 小型试验场限制翻转；此模型不承担完整翻车动力学。 */
	if (fabsf(v->pitch) > 0.60f)
	{
		v->pitch = copysignf(0.60f, v->pitch);
		v->pitchRate = 0;
	}
	if (fabsf(v->roll) > 0.60f)
	{
		v->roll = copysignf(0.60f, v->roll);
		v->rollRate = 0;
	}
	YMPME_Vehicle_UpdateSupports(v, terrain);
	return isfinite(v->y) && isfinite(v->pitch) && isfinite(v->roll);
}
