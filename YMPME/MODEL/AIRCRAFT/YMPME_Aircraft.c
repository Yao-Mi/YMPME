#include "YMPME_Aircraft.h"
#include <string.h>
#define PI 3.14159265358979323846f
pme_vec3 YMPME_Aircraft_Rotate(const pme_aircraft *a, pme_vec3 v)
{
	pme_vec3 u = YMPME_Vec3(a->q[0], a->q[1], a->q[2]);
	pme_vec3 t = YMPME_VecScale(YMPME_VecCross(u, v), 2);
	return YMPME_VecAdd(v, YMPME_VecAdd(YMPME_VecScale(t, a->q[3]), YMPME_VecCross(u, t)));
}
static pme_vec3 unrotate(const pme_aircraft *a, pme_vec3 v)
{
	return YMPME_Vec3(YMPME_VecDot(v, YMPME_Aircraft_Rotate(a, YMPME_Vec3(1, 0, 0))),
					  YMPME_VecDot(v, YMPME_Aircraft_Rotate(a, YMPME_Vec3(0, 1, 0))),
					  YMPME_VecDot(v, YMPME_Aircraft_Rotate(a, YMPME_Vec3(0, 0, 1))));
}
pme_result YMPME_Aircraft_Init(pme_aircraft *a, pme_aircraft_kind kind, pme_vec3 p)
{
	if (!a || (kind != PME_AIRCRAFT_DRONE && kind != PME_AIRCRAFT_EAGLE) || !YMPME_VecFinite(p) ||
		p.y < .3f)
		return PME_INVALID_ARGUMENT;
	memset(a, 0, sizeof(*a));
	a->kind = kind;
	a->position = p;
	a->q[3] = 1;
	a->mass = kind == PME_AIRCRAFT_DRONE ? 1.6f : 3.0f;
	a->inertia =
		kind == PME_AIRCRAFT_DRONE ? YMPME_Vec3(.14f, .25f, .14f) : YMPME_Vec3(.3f, .6f, .4f);
	if (kind == PME_AIRCRAFT_DRONE)
		for (int i = 0; i < 4; ++i)
			a->rotorSpeed[i] = sqrtf(a->mass * 9.81f / (4 * .0001f));
	else
	{
		a->velocity.z = 10;
		a->q[0] = sinf(-.08f * .5f);
		a->q[3] = cosf(-.08f * .5f);
	}
	return PME_OK;
}
static pme_vec3 attitudeTorque(const pme_aircraft *a, float pitch, float roll, float yawRate)
{
	pme_vec3 f = YMPME_Aircraft_Rotate(a, YMPME_Vec3(0, 0, 1));
	pme_vec3 r = YMPME_Aircraft_Rotate(a, YMPME_Vec3(1, 0, 0));
	pme_vec3 u = YMPME_Aircraft_Rotate(a, YMPME_Vec3(0, 1, 0));
	float p = asinf(YMPME_Clamp(-f.y, -1, 1)), b = atan2f(r.y, u.y);
	/* Y-X-Z姿态的欧拉角速率转机体系角速度，转弯时补偿俯仰/滚转耦合。 */
	float pd = 4 * (pitch - p), bd = 4 * (roll - b);
	pme_vec3 target =
		YMPME_Vec3(pd * cosf(b) + yawRate * sinf(b) * cosf(p),
				   -pd * sinf(b) + yawRate * cosf(b) * cosf(p), bd - yawRate * sinf(p));
	pme_vec3 w = a->angularVelocity;
	pme_vec3 torque =
		YMPME_Vec3(a->inertia.x * 10 * (target.x - w.x), a->inertia.y * 6 * (target.y - w.y),
				   a->inertia.z * 10 * (target.z - w.z));
	return YMPME_VecAdd(torque, YMPME_VecCross(w, YMPME_Vec3(a->inertia.x * w.x, a->inertia.y * w.y,
															 a->inertia.z * w.z)));
}
static int drone(pme_aircraft *a, const pme_aircraft_command *c, pme_vec3 wind, float dt,
				 const float *motor)
{
	pme_vec3 torque = {0, 0, 0};
	float total = 0;
	if (!motor)
	{
		pme_vec3 f = YMPME_Aircraft_Rotate(a, YMPME_Vec3(0, 0, 1));
		float yaw = atan2f(f.x, f.z), sn = sinf(yaw), cs = cosf(yaw);
		float vx = 5 * (c->forward * sn + c->right * cs),
			  vz = 5 * (c->forward * cs - c->right * sn);
		float ax = YMPME_Clamp(2 * (vx - a->velocity.x), -5, 5),
			  az = YMPME_Clamp(2 * (vz - a->velocity.z), -5, 5);
		float pitch = YMPME_Clamp(atan2f(ax * sn + az * cs, 9.81f), -.45f, .45f);
		float roll = YMPME_Clamp(-atan2f(ax * cs - az * sn, 9.81f), -.45f, .45f);
		torque = attitudeTorque(a, pitch, roll, c->turn * 1.2f);
		pme_vec3 up = YMPME_Aircraft_Rotate(a, YMPME_Vec3(0, 1, 0));
		total = a->mass * (9.81f + YMPME_Clamp(4 * (c->climb * 2.5f - a->velocity.y), -6, 8)) /
				fmaxf(.5f, up.y);
	}
	pme_flight_load sum = YMPME_Flight_ZeroLoad();
	a->thrust = 0;
	for (int i = 0; i < 4; ++i)
	{
		float x = (i & 1) ? .45f : -.45f, z = (i & 2) ? .45f : -.45f;
		float spin = ((i == 0 || i == 3) ? 1.0f : -1.0f);
		float thrust =
			YMPME_Clamp(total / 4 - z * torque.x / (4 * .45f * .45f) +
							x * torque.z / (4 * .45f * .45f) - spin * torque.y / (4 * .03f),
						0, 11) *
			c->power;
		float speed = motor ? motor[i] * sqrtf(11 / .0001f) : sqrtf(thrust / .0001f);
		a->rotorSpeed[i] += (speed - a->rotorSpeed[i]) * (1 - expf(-dt / .06f));
		a->rotorPhase[i] = remainderf(a->rotorPhase[i] + spin * a->rotorSpeed[i] * dt, 2 * PI);
		pme_rotor_desc rotor = {YMPME_Vec3(x, 0, z), YMPME_Vec3(0, 1, 0), .0001f, .000003f};
		pme_flight_load load;
		if (YMPME_Flight_RotorLoad(&rotor, spin * a->rotorSpeed[i], &load) != PME_OK ||
			YMPME_Flight_AddLoads(&sum, &load, &sum) != PME_OK)
			return 0;
		a->thrust += load.force.y;
	}
	a->load.force = YMPME_VecSub(YMPME_Aircraft_Rotate(a, sum.force),
								 YMPME_VecScale(YMPME_VecSub(a->velocity, wind), .3f));
	a->load.torque = YMPME_Aircraft_Rotate(
		a, YMPME_VecSub(sum.torque, YMPME_VecScale(a->angularVelocity, .025f)));
	a->lift = a->load.force.y;
	return 1;
}
static int eagle(pme_aircraft *a, const pme_aircraft_command *c, pme_vec3 wind, float dt)
{
	pme_vec3 air = YMPME_VecSub(a->velocity, wind), local = unrotate(a, air);
	float alpha = -atan2f(local.y, fmaxf(.2f, local.z));
	float cl = YMPME_Clamp(.25f + 4.2f * alpha, -.5f, 1.35f);
	if (fabsf(alpha) > .35f)
		cl *= fmaxf(.15f, 1 - (fabsf(alpha) - .35f) * 1.3f);
	float cd = .03f + .06f * cl * cl + .5f * sinf(alpha) * sinf(alpha);
	pme_vec3 up = YMPME_Aircraft_Rotate(a, YMPME_Vec3(0, 1, 0));
	pme_aero_desc wing = {YMPME_Vec3(0, 0, 0), up, .85f, cl, cd};
	/* 极端垂直来流时退化为纯阻力，升力方向不唯一。 */
	if (YMPME_VecDot(YMPME_VecCross(air, up), YMPME_VecCross(air, up)) < 1e-8f)
		wing.liftCoefficient = 0;
	pme_flight_load load;
	if (YMPME_Flight_AeroLoad(&wing, air, 1.225f, &load) != PME_OK)
		return 0;
	a->wingPhase = remainderf(a->wingPhase + 2 * PI * 2.2f * dt, 2 * PI);
	float targetAngle = c->power * .55f * sinf(a->wingPhase);
	a->wingAngle += (targetAngle - a->wingAngle) * (1 - expf(-dt * 24));
	a->thrust = c->power * (4.4f + 1.4f * c->forward) * (1 + .45f * cosf(a->wingPhase));
	pme_vec3 forward = YMPME_Aircraft_Rotate(a, YMPME_Vec3(0, 0, 1));
	a->load.force = YMPME_VecAdd(load.force, YMPME_VecScale(forward, a->thrust));
	float roll = YMPME_Clamp(-.55f * (c->turn + c->right), -.65f, .65f);
	/* 低空速时降低机头，避免高度指令持续拉升而失速。 */
	float pitch = -.08f - .18f * c->climb + YMPME_Clamp((9.5f - a->airspeed) * .12f, 0, .6f);
	float yawRate = -9.81f * tanf(roll) / fmaxf(5, a->airspeed);
	a->load.torque = YMPME_Aircraft_Rotate(a, attitudeTorque(a, pitch, roll, yawRate));
	a->lift = .5f * 1.225f * a->airspeed * a->airspeed * wing.area * cl;
	return 1;
}
static pme_result stepAircraft(pme_aircraft *aircraft, const pme_aircraft_command *c, pme_vec3 wind,
							   float dt, const float *motor)
{
	if (!aircraft || !c || !YMPME_VecFinite(wind) || !isfinite(dt) || dt <= 0 || dt > 1.0f / 120 ||
		!isfinite(c->forward) || fabsf(c->forward) > 1 || !isfinite(c->right) ||
		fabsf(c->right) > 1 || !isfinite(c->climb) || fabsf(c->climb) > 1 || !isfinite(c->turn) ||
		fabsf(c->turn) > 1 || !isfinite(c->power) || c->power < 0 || c->power > 1)
		return PME_INVALID_ARGUMENT;
	pme_aircraft a = *aircraft;
	if ((a.kind != PME_AIRCRAFT_DRONE && a.kind != PME_AIRCRAFT_EAGLE) || !isfinite(a.mass) ||
		a.mass <= 0 || !YMPME_VecFinite(a.position) || !YMPME_VecFinite(a.velocity) ||
		!YMPME_VecFinite(a.angularVelocity) || !YMPME_VecFinite(a.inertia) || a.inertia.x <= 0 ||
		a.inertia.y <= 0 || a.inertia.z <= 0)
		return PME_INVALID_ARGUMENT;
	pme_vec3 air = YMPME_VecSub(a.velocity, wind);
	a.airspeed = sqrtf(YMPME_VecDot(air, air));
	if (!(a.kind == PME_AIRCRAFT_DRONE ? drone(&a, c, wind, dt, motor) : eagle(&a, c, wind, dt)))
		return PME_INVALID_ARGUMENT;
	pme_vec3 torque = unrotate(&a, a.load.torque), w = a.angularVelocity;
	pme_vec3 momentum = YMPME_Vec3(w.x * a.inertia.x, w.y * a.inertia.y, w.z * a.inertia.z);
	torque = YMPME_VecSub(torque, YMPME_VecCross(w, momentum));
	w = YMPME_VecAdd(w, YMPME_Vec3(torque.x / a.inertia.x * dt, torque.y / a.inertia.y * dt,
								   torque.z / a.inertia.z * dt));
	a.angularVelocity = w;
	float x = a.q[0], y = a.q[1], z = a.q[2], s = a.q[3];
	a.q[0] += .5f * dt * (s * w.x + y * w.z - z * w.y);
	a.q[1] += .5f * dt * (s * w.y + z * w.x - x * w.z);
	a.q[2] += .5f * dt * (s * w.z + x * w.y - y * w.x);
	a.q[3] += -.5f * dt * (x * w.x + y * w.y + z * w.z);
	float norm = sqrtf(a.q[0] * a.q[0] + a.q[1] * a.q[1] + a.q[2] * a.q[2] + a.q[3] * a.q[3]);
	if (!isfinite(norm) || norm < 1e-8f)
		return PME_INVALID_ARGUMENT;
	for (int i = 0; i < 4; ++i)
		a.q[i] /= norm;
	pme_vec3 acc = YMPME_VecScale(a.load.force, 1 / a.mass);
	acc.y -= 9.81f;
	a.velocity = YMPME_VecAdd(a.velocity, YMPME_VecScale(acc, dt));
	a.position = YMPME_VecAdd(a.position, YMPME_VecScale(a.velocity, dt));
	a.grounded = a.position.y < .3f;
	if (a.grounded)
	{
		a.position.y = .3f;
		a.velocity.y = fmaxf(0, a.velocity.y);
		a.velocity.x *= expf(-dt * 3);
		a.velocity.z *= expf(-dt * 3);
		a.angularVelocity = YMPME_VecScale(a.angularVelocity, expf(-dt * 8));
	}
	a.lastStep = dt;
	a.acceleration = YMPME_VecScale(YMPME_VecSub(a.velocity, aircraft->velocity), 1 / dt);
	a.angularAcceleration =
		YMPME_VecScale(YMPME_VecSub(YMPME_Aircraft_Rotate(&a, a.angularVelocity),
									YMPME_Aircraft_Rotate(aircraft, aircraft->angularVelocity)),
					   1 / dt);
	a.time += dt;
	if (!YMPME_VecFinite(a.position) || !YMPME_VecFinite(a.velocity) ||
		!YMPME_VecFinite(a.angularVelocity) || !isfinite(a.time))
		return PME_INVALID_ARGUMENT;
	*aircraft = a;
	return PME_OK;
}

pme_result YMPME_Aircraft_Step(pme_aircraft *a, const pme_aircraft_command *c, pme_vec3 wind,
							   float dt)
{
	return stepAircraft(a, c, wind, dt, NULL);
}
pme_result YMPME_Aircraft_StepMotors(pme_aircraft *a, const float motor[4], pme_vec3 wind, float dt)
{
	if (!a || a->kind != PME_AIRCRAFT_DRONE || !motor)
		return PME_INVALID_ARGUMENT;
	for (int i = 0; i < 4; ++i)
		if (!isfinite(motor[i]) || motor[i] < 0 || motor[i] > 1)
			return PME_INVALID_ARGUMENT;
	pme_aircraft_command c = {0, 0, 0, 0, 0};
	return stepAircraft(a, &c, wind, dt, motor);
}
pme_result YMPME_Aircraft_GetFeedback(const pme_aircraft *a, pme_feedback_state *out)
{
	if (!a || !out || !isfinite(a->time) || !YMPME_VecFinite(a->position))
		return PME_INVALID_ARGUMENT;
	pme_feedback_state f = {0};
	f.time = a->time;
	f.interval = a->lastStep;
	f.position = a->position;
	f.velocity = a->velocity;
	f.acceleration = a->acceleration;
	f.angularVelocity = YMPME_Aircraft_Rotate(a, a->angularVelocity);
	f.angularAcceleration = a->angularAcceleration;
	f.rotation.axis[0] = YMPME_Aircraft_Rotate(a, YMPME_Vec3(1, 0, 0));
	f.rotation.axis[1] = YMPME_Aircraft_Rotate(a, YMPME_Vec3(0, 1, 0));
	f.rotation.axis[2] = YMPME_Aircraft_Rotate(a, YMPME_Vec3(0, 0, 1));
	if (!YMPME_Rotation_IsValid(f.rotation))
		return PME_INVALID_ARGUMENT;
	f.valid = PME_FEEDBACK_POSE | PME_FEEDBACK_VELOCITY | PME_FEEDBACK_ANGULAR_VELOCITY;
	if (a->lastStep > 0)
		f.valid |= PME_FEEDBACK_ACCELERATION | PME_FEEDBACK_ANGULAR_ACCELERATION;
	*out = f;
	return PME_OK;
}
