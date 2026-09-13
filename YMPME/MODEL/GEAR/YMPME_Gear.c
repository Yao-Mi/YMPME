#include "YMPME_Gear.h"
#include <string.h>
static void point(pme_gear_point *p, int *n, double r, double angle)
{
	p[*n] = (pme_gear_point){r * cos(angle), r * sin(angle)};
	++*n;
}
static double halfWidth(int z, double base, double r, double inv)
{
	double a = acos(base / r);
	return PME_GEAR_PI / (2 * z) + inv - (tan(a) - a) - .025 / z;
}
int YMPME_Gear_Profile(int z, double m, pme_gear_point *out, int capacity)
{
	int tooth, j, n = 0;
	double pitch = m * z * .5, base = pitch * cos(20 * PME_GEAR_PI / 180), root = pitch - 1.25 * m,
		   tip = pitch + m;
	double a = 20 * PME_GEAR_PI / 180, inv = tan(a) - a;
	if (!out || z < 18 || z > 40 || !isfinite(m) || m <= 0 || capacity < z * 23)
		return 0;
	for (tooth = 0; tooth < z; ++tooth)
	{
		double center = 2 * PME_GEAR_PI * tooth / z, h = halfWidth(z, base, base, inv),
			   ht = halfWidth(z, base, tip, inv);
		point(out, &n, root, center - PME_GEAR_PI / z);
		/* 略向齿槽偏置齿根连接，保持轮廓严格星形，顶面扇形三角无退化。 */
		point(out, &n, root, center - h - .006);
		point(out, &n, base, center - h);
		for (j = 1; j <= 8; ++j)
		{
			double r = base + (tip - base) * j / 8;
			point(out, &n, r, center - halfWidth(z, base, r, inv));
		}
		for (j = 1; j <= 3; ++j)
			point(out, &n, tip, center - ht + 2 * ht * j / 3);
		for (j = 7; j >= 0; --j)
		{
			double r = base + (tip - base) * j / 8;
			point(out, &n, r, center + halfWidth(z, base, r, inv));
		}
		point(out, &n, root, center + h + .006);
	}
	return n;
}
void YMPME_Gear_Reset(pme_gear_pair *p)
{
	YMPME_Gear_ResetMode(p, PME_GEAR_CONSTRAINT);
}
void YMPME_Gear_ResetMode(pme_gear_pair *p, pme_gear_mode mode)
{
	if (!p || mode < PME_GEAR_CONSTRAINT || mode > PME_GEAR_BACKLASH)
		return;
	memset(p, 0, sizeof(*p));
	p->mode = mode;
	p->target = 12 * 2 * PME_GEAR_PI / 60;
	p->motor = 1;
	p->angleB = PME_GEAR_PI - PME_GEAR_PI / 30;
	p->distance = mode == PME_GEAR_BACKLASH ? 3.08 : 3.0;
	p->loadTorque = .6;
	if (mode != PME_GEAR_CONSTRAINT)
		p->profileReady = YMPME_Gear_Profile(20, .12, p->profile[0], PME_GEAR_PROFILE_MAX) &&
						  YMPME_Gear_Profile(30, .12, p->profile[1], PME_GEAR_PROFILE_MAX);
}
double YMPME_Gear_DrivenOmega(const pme_gear_pair *p)
{
	return p->mode == PME_GEAR_CONSTRAINT ? -p->omega * 2 / 3 : p->omegaB;
}
int YMPME_Gear_Step(pme_gear_pair *p, double dt)
{
	const double ratio = 20.0 / 30.0;
	double inertia = .8 + 2.4 * ratio * ratio;
	if (!p || !isfinite(dt) || dt <= 0 || dt > .05 || !isfinite(p->target))
		return 0;
	if (p->mode != PME_GEAR_CONSTRAINT)
		return YMPME_Gear_CollisionStep(p, dt);
	/* 固定轴理想约束，输出惯量按传动比平方折算到输入端。 */
	p->torque = p->motor ? fmax(-8, fmin(8, 10 * (p->target - p->omega))) : 0;
	p->omega += (p->torque - (.06 + .06 * ratio * ratio) * p->omega) / inertia * dt;
	p->omega = copysign(fmax(0, fabs(p->omega) - p->loadTorque * ratio / inertia * dt), p->omega);
	p->angle += p->omega * dt;
	/* 输入转三圈、输出转两圈后才一起回绕，保留彩色标记的连续性。 */
	p->angle = remainder(p->angle, 6 * PME_GEAR_PI);
	return isfinite(p->angle) && isfinite(p->omega);
}
double YMPME_Gear_DrivenAngle(const pme_gear_pair *p)
{
	return p->mode == PME_GEAR_CONSTRAINT ? PME_GEAR_PI - PME_GEAR_PI / 30 - p->angle * (20.0 / 30.0)
										  : p->angleB;
}
