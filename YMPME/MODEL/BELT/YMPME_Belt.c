#include "YMPME_Belt.h"
#include <string.h>
static pme_belt_point add(pme_belt_point a, pme_belt_point b)
{
	return (pme_belt_point){a.x + b.x, a.y + b.y};
}
static pme_belt_point sub(pme_belt_point a, pme_belt_point b)
{
	return (pme_belt_point){a.x - b.x, a.y - b.y};
}
static pme_belt_point scale(pme_belt_point a, double s)
{
	return (pme_belt_point){a.x * s, a.y * s};
}
static double dot(pme_belt_point a, pme_belt_point b)
{
	return a.x * b.x + a.y * b.y;
}
static double clamp(double x, double a, double b)
{
	return fmax(a, fmin(b, x));
}
double YMPME_Belt_Length(const pme_belt *b)
{
	double ra = b->radius[0] + b->nodeRadius, rb = b->radius[1] + b->nodeRadius;
	double theta = acos((ra - rb) / b->distance),
		   line = sqrt(b->distance * b->distance - (rb - ra) * (rb - ra));
	return 2 * line + 2 * theta * rb + (2 * PME_BELT_PI - 2 * theta) * ra;
}
pme_belt_point YMPME_Belt_Path(const pme_belt *b, double s)
{
	double ra = b->radius[0] + b->nodeRadius, rb = b->radius[1] + b->nodeRadius;
	double theta = acos((ra - rb) / b->distance), nx = cos(theta), ny = sin(theta);
	double line = sqrt(b->distance * b->distance - (rb - ra) * (rb - ra)), total = YMPME_Belt_Length(b), t;
	pme_belt_point au = {-b->distance / 2 + ra * nx, ra * ny}, bu = {b->distance / 2 + rb * nx, rb * ny};
	s = fmod(s, total);
	if (s < 0)
		s += total;
	if (s < line)
		return add(au, scale(sub(bu, au), s / line));
	s -= line;
	if (s < 2 * theta * rb)
	{
		t = theta - s / rb;
		return (pme_belt_point){b->distance / 2 + rb * cos(t), rb * sin(t)};
	}
	s -= 2 * theta * rb;
	au.y = -au.y;
	bu.y = -bu.y;
	if (s < line)
		return add(bu, scale(sub(au, bu), s / line));
	s -= line;
	t = -theta - s / ra;
	return (pme_belt_point){-b->distance / 2 + ra * cos(t), ra * sin(t)};
}
void YMPME_Belt_Init(pme_belt *b, int collision)
{
	if (!b || (collision != 0 && collision != 1))
		return;
	int i;
	memset(b, 0, sizeof(*b));
	b->collision = collision;
	b->motor = 1;
	b->target = -1.0;
	b->mu = .85;
	b->radius[0] = .65;
	b->radius[1] = .95;
	b->inertia[0] = .12;
	b->inertia[1] = .25;
	b->distance = 2.8;
	b->nodeRadius = .035;
	for (i = 0; i < PME_BELT_N; ++i)
	{
		b->p[i] = YMPME_Belt_Path(b, i * YMPME_Belt_Length(b) / PME_BELT_N);
		b->contactWheel[i] = -1;
	}
	for (i = 0; i < PME_BELT_N; ++i)
	{
		pme_belt_point d = sub(b->p[(i + 1) % PME_BELT_N], b->p[i]);
		b->rest[i] = hypot(d.x, d.y) * .997;
	}
}
void YMPME_Belt_Step(pme_belt *b, double dt)
{
	if (!b || !isfinite(dt) || dt <= 0 || dt > .01 || !isfinite(b->target) || !isfinite(b->mu) || b->mu < 0)
		return;
	int i, j, k;
	double ratio = b->radius[0] / b->radius[1];
	double torque = b->motor ? clamp(2.0 * (b->target - b->omega[0]), -1.5, 1.5) : 0;
	if (!b->collision)
	{
		double inertia =
			b->inertia[0] + b->inertia[1] * ratio * ratio + PME_BELT_N * .018 * b->radius[0] * b->radius[0];
		b->omega[0] += (torque - .015 * b->omega[0]) / inertia * dt;
		b->omega[0] = copysign(fmax(0, fabs(b->omega[0]) - .045 * ratio / inertia * dt), b->omega[0]);
		b->omega[1] = b->omega[0] * ratio;
		for (i = 0; i < 2; ++i)
			b->angle[i] = remainder(b->angle[i] + b->omega[i] * dt, 2 * PME_BELT_PI);
		/* 皮带标记相位独立累积，不能在轮子转一圈时回绕。 */
		b->phase = fmod(b->phase - b->radius[0] * b->omega[0] * dt, YMPME_Belt_Length(b));
		for (i = 0; i < PME_BELT_N; ++i)
			b->p[i] = YMPME_Belt_Path(b, i * YMPME_Belt_Length(b) / PME_BELT_N + b->phase);
		return;
	}
	b->omega[0] += (torque - .008 * b->omega[0]) / b->inertia[0] * dt;
	b->omega[1] *= exp(-.008 / b->inertia[1] * dt);
	b->omega[1] = copysign(fmax(0, fabs(b->omega[1]) - .045 / b->inertia[1] * dt), b->omega[1]);
	for (i = 0; i < PME_BELT_N; ++i)
	{
		b->old[i] = b->p[i];
		b->v[i].y -= 9.81 * dt;
		b->p[i] = add(b->p[i], scale(b->v[i], dt));
		b->lambda[i] = 0;
		b->normalImpulse[i] = 0;
		b->contactWheel[i] = -1;
	}
	/* XPBD distance constraints followed by unilateral wheel contacts.
	   Normal corrections estimate the impulse budget for Coulomb friction. */
	for (k = 0; k < 24; ++k)
	{
		for (j = 0; j < PME_BELT_N; ++j)
		{
			i = k % 2 ? PME_BELT_N - 1 - j : j;
			int next = (i + 1) % PME_BELT_N;
			pme_belt_point d = sub(b->p[next], b->p[i]);
			double len = hypot(d.x, d.y), alpha = 1e-7 / (dt * dt);
			if (len > 1e-12)
			{
				double dl = (-(len - b->rest[i]) - alpha * b->lambda[i]) / (2 / .018 + alpha);
				b->lambda[i] += dl;
				pme_belt_point change = scale(d, dl / (.018 * len));
				b->p[i] = sub(b->p[i], change);
				b->p[next] = add(b->p[next], change);
			}
		}
		for (i = 0; i < PME_BELT_N; ++i)
			for (j = 0; j < 2; ++j)
			{
				pme_belt_point center = {(j ? 1 : -1) * b->distance / 2, 0}, d = sub(b->p[i], center);
				double len = hypot(d.x, d.y), r = b->radius[j] + b->nodeRadius;
				if (len < r && len > 1e-12)
				{
					b->p[i] = add(center, scale(d, r / len));
					b->normalImpulse[i] += (r - len) * .018 / dt;
					b->contactWheel[i] = j;
				}
			}
	}
	b->contacts = 0;
	b->contactTorque = 0;
	b->maxStretch = 0;
	for (i = 0; i < PME_BELT_N; ++i)
	{
		b->v[i] = scale(sub(b->p[i], b->old[i]), 1 / dt);
		j = b->contactWheel[i];
		if (j >= 0)
		{
			pme_belt_point d = sub(b->p[i], (pme_belt_point){(j ? 1 : -1) * b->distance / 2, 0});
			double len = hypot(d.x, d.y);
			pme_belt_point tangent = {-d.y / len, d.x / len};
			double r = b->radius[j];
			double relative = dot(b->v[i], tangent) - b->omega[j] * r;
			double impulse = clamp(-relative / (1 / .018 + r * r / b->inertia[j]),
								   -b->mu * b->normalImpulse[i], b->mu * b->normalImpulse[i]);
			b->v[i] = add(b->v[i], scale(tangent, impulse / .018));
			b->omega[j] -= r * impulse / b->inertia[j];
			if (j == 1)
				b->contactTorque -= r * impulse / dt;
			++b->contacts;
		}
		pme_belt_point d = sub(b->p[(i + 1) % PME_BELT_N], b->p[i]);
		b->maxStretch = fmax(b->maxStretch, fabs(hypot(d.x, d.y) / b->rest[i] - 1));
	}
	for (i = 0; i < 2; ++i)
		b->angle[i] = remainder(b->angle[i] + b->omega[i] * dt, 2 * PME_BELT_PI);
}
