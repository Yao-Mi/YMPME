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
static pme_gear_point transform(pme_gear_point p, double a, double x)
{
	return (pme_gear_point){p.x * cos(a) - p.y * sin(a) + x, p.x * sin(a) + p.y * cos(a)};
}
static int inside(pme_gear_point p, const pme_gear_point *poly, int n)
{
	int i, j = n - 1, c = 0;
	for (i = 0; i < n; j = i++)
		if (((poly[i].y > p.y) != (poly[j].y > p.y)) &&
		    p.x < (poly[j].x - poly[i].x) * (p.y - poly[i].y) / (poly[j].y - poly[i].y) + poly[i].x)
			c = !c;
	return c;
}
int main(void)
{
	pme_gear_pair p;
	pme_gear_point a[PME_GEAR_PROFILE_MAX], b[PME_GEAR_PROFILE_MAX], A[PME_GEAR_PROFILE_MAX], B[PME_GEAR_PROFILE_MAX];
	int n = YMPME_Gear_Profile(20, .12, a, PME_GEAR_PROFILE_MAX), m = YMPME_Gear_Profile(30, .12, b, PME_GEAR_PROFILE_MAX), i, k;
	CHECK(n == 460 && m == 690);
	CHECK(!YMPME_Gear_Profile(20, .12, a, 10));
	YMPME_Gear_Reset(&p);
	p.loadTorque = 0;
	CHECK(!YMPME_Gear_Step(&p, 0));
	for (i = 0; i < 1200; ++i)
		CHECK(YMPME_Gear_Step(&p, 1.0 / 120));
	CHECK(fabs(p.omega - p.target) < .02);
	for (i = 0; i < 10000; ++i)
	{
		double before = YMPME_Gear_DrivenAngle(&p);
		CHECK(YMPME_Gear_Step(&p, 1.0 / 120));
		CHECK(fabs(remainder(YMPME_Gear_DrivenAngle(&p) - before, 2 * PME_GEAR_PI) + p.omega / 120 * 2 / 3) < 1e-10);
	}
	p.motor = 0;
	{
		double w = p.omega;
		CHECK(YMPME_Gear_Step(&p, 1.0 / 120));
		CHECK(p.omega > 0 && p.omega < w && p.torque == 0);
	}
	p.motor = 1;
	p.target = -p.target;
	for (i = 0; i < 1200; ++i)
		CHECK(YMPME_Gear_Step(&p, 1.0 / 120));
	CHECK(p.omega < 0);
	/* 跨一个完整啮合周期检查双向轮廓采样，防止相位错误和肉眼难见的齿形穿插。 */
	for (k = 0; k < 120; ++k)
	{
		p.angle = k * (2 * PME_GEAR_PI / 20) / 120;
		for (i = 0; i < n; ++i)
			A[i] = transform(a[i], p.angle, 0);
		for (i = 0; i < m; ++i)
			B[i] = transform(b[i], YMPME_Gear_DrivenAngle(&p), 3);
		for (i = 0; i < n; ++i)
			if (A[i].x > 1.05)
				CHECK(!inside(A[i], B, m));
		for (i = 0; i < m; ++i)
			if (B[i].x < 1.33)
				CHECK(!inside(B[i], A, n));
	}
	puts("PASS involute outline, tooth phase, meshing clearance, ratio, wrap, motor inertia, coast and "
	     "reverse");
	return 0;
}
