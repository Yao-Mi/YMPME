#include "MODEL/GEAR/YMPME_Gear.h"
#include <stdio.h>
#define CHECK(x)                                                                                             \
	do                                                                                                       \
	{                                                                                                        \
		if (!(x))                                                                                            \
		{                                                                                                    \
			fprintf(stderr, "line %d: %s\n", __LINE__, #x);                                                  \
			return 1;                                                                                        \
		}                                                                                                    \
	} while (0)
static double phase(const pme_gear_pair *p)
{
	return remainder(p->angleB + p->angle * 2 / 3 - (PME_GEAR_PI - PME_GEAR_PI / 30), 2 * PME_GEAR_PI / 30);
}
int main(void)
{
	pme_gear_pair p;
	int mode, i, hits;
	double gaps[2], penetration = 0;
	for (mode = 1; mode <= 2; ++mode)
	{
		double start, error = 0;
		YMPME_Gear_ResetMode(&p, (pme_gear_mode)mode);
		hits = 0;
		for (i = 0; i < 600; ++i)
		{
			CHECK(YMPME_Gear_Step(&p, 1.0 / 120));
			hits += p.contacts > 0;
			penetration = fmax(penetration, p.maxPenetration);
		}
		CHECK(hits > 100 && p.omega > 1 && p.omegaB < -.7);
		CHECK(fabs(p.omegaB + p.omega * 2 / 3) < .03);
		start = phase(&p);
		p.target = -p.target;
		for (i = 0; i < 300; ++i)
		{
			CHECK(YMPME_Gear_Step(&p, 1.0 / 120));
			if (p.contactTorque > .001)
				break;
		}
		CHECK(i < 300);
		gaps[mode - 1] = fabs(phase(&p) - start) * 1.5;
		for (i = 0; i < 600; ++i)
			CHECK(YMPME_Gear_Step(&p, 1.0 / 120));
		CHECK(p.omega < -1 && p.omegaB > .7);
		/* 最高界面速度下长期传动，用未回绕的相对角累计检测跳齿。 */
		p.target = 60 * 2 * PME_GEAR_PI / 60;
		for (i = 0; i < 1200; ++i)
		{
			double a = p.angle, b = p.angleB;
			CHECK(YMPME_Gear_Step(&p, 1.0 / 120));
			error += remainder(p.angle - a, 2 * PME_GEAR_PI) * 2 / 3 + remainder(p.angleB - b, 2 * PME_GEAR_PI);
			CHECK(fabs(error) < .09);
			penetration = fmax(penetration, p.maxPenetration);
		}
		CHECK(p.omega > 5 && p.omegaB < -3.3);
		/* 移开到齿顶圆不相交：输入自转，输出必须完全静止。 */
		YMPME_Gear_ResetMode(&p, (pme_gear_mode)mode);
		p.distance = 3.5;
		for (i = 0; i < 240; ++i)
		{
			CHECK(YMPME_Gear_Step(&p, 1.0 / 120));
			CHECK(!p.contacts && p.contactTorque == 0);
		}
		CHECK(p.omega > 1 && p.omegaB == 0 && p.angleB == PME_GEAR_PI - PME_GEAR_PI / 30);
		/* 反向施加初速度：输出轮也能通过接触带动输入轮，且没有凭空增能。 */
		YMPME_Gear_ResetMode(&p, (pme_gear_mode)mode);
		p.motor = 0;
		p.loadTorque = 0;
		p.omegaB = 1;
		for (i = 0; i < 240; ++i)
		{
			CHECK(YMPME_Gear_Step(&p, 1.0 / 120));
			CHECK(.4 * p.omega * p.omega + 1.2 * p.omegaB * p.omegaB <= 1.20001);
		}
		CHECK(p.omega < -.2 && p.omegaB > .1);
	}
	CHECK(gaps[0] > .0005 && gaps[0] < .012);
	CHECK(gaps[1] > .03 && gaps[1] > gaps[0] * 5);
	CHECK(penetration < .01);
	printf("PASS independent tooth impulses, separation, reverse drive, energy, 60 RPM and backlash %.3f / "
	       "%.3f deg; max penetration %.6fm\n",
	       gaps[0] * 180 / PME_GEAR_PI, gaps[1] * 180 / PME_GEAR_PI, penetration);
	return 0;
}
