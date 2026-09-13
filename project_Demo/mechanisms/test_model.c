#include "model.h"
#include <stdio.h>
#include <string.h>
#define CHECK(c)                                                                                             \
	do                                                                                                       \
	{                                                                                                        \
		if (!(c))                                                                                            \
		{                                                                                                    \
			fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #c);                                             \
			return 1;                                                                                        \
		}                                                                                                    \
	} while (0)
static double energy(const swing_model *s)
{
	return .5 * s->mass * YMPME_VecDot(s->velocity, s->velocity) +
		   s->mass * 9.81 * (s->position.y - s->anchor.y + s->length);
}
int main(void)
{
	mechanisms_model m;
	pme_belt frictionless;
	Mechanisms_Init(&m);
	YMPME_Belt_Init(&frictionless, 1);
	frictionless.mu = 0;
	CHECK(!memcmp(&m.swing.position, &m.swing.end, sizeof(pme_vec3)));
	CHECK(YMPME_VecDot(m.swing.velocity, m.swing.velocity) == 0);
	double e0 = energy(&m.swing), left = 10, right = -10;
	for (int i = 0; i < 1440; ++i)
	{
		CHECK(Mechanisms_Step(&m, 0, 1.0 / 120));
		for (int k = 0; k < 5; ++k)
			YMPME_Belt_Step(&frictionless, 1.0 / 600);
		CHECK(fabs(m.belt[0].omega[1] - m.belt[0].omega[0] * .65 / .95) < 1e-12);
		CHECK(fabs(4 * m.hoist.loadY + m.hoist.freeY - m.hoist.constant) < 1e-12);
		pme_vec3 d = YMPME_VecSub(m.swing.position, m.swing.anchor);
		CHECK(sqrt(YMPME_VecDot(d, d)) <= m.swing.length + 1e-6);
		CHECK(energy(&m.swing) < e0 * 1.02);
		left = fmin(left, m.swing.position.x);
		right = fmax(right, m.swing.position.x);
		if (i == 599)
		{
			CHECK(m.belt[1].omega[0] < -.8 && m.belt[1].omega[1] < -.5);
			m.belt[1].target = 1;
		}
		CHECK(m.belt[1].maxStretch < .035);
		for (int n = 0; n < PME_BELT_N; ++n)
			for (int w = 0; w < 2; ++w)
				CHECK(hypot(m.belt[1].p[n].x - (w ? 1 : -1) * 1.4, m.belt[1].p[n].y) >=
					  m.belt[1].radius[w] + m.belt[1].nodeRadius - 1e-8);
	}
	CHECK(m.belt[1].omega[0] > .8 && m.belt[1].omega[1] > .5);
	CHECK(m.belt[1].contacts > 20 && frictionless.contacts > 20);
	CHECK(frictionless.omega[0] < -.8 && fabs(frictionless.omega[1]) < 1e-12);
	CHECK(left < -1.2 && right > 1.2);
	CHECK(fabs(m.hoist.tension - 8 * 9.81 / 4) < 1e-6);
	double load = m.hoist.loadY, freeEnd = m.hoist.freeY;
	for (int i = 0; i < 600; ++i)
		YMPME_Pulley_Step(&m.hoist, 1, 1.0 / 600);
	CHECK(m.hoist.loadY > load + .1);
	CHECK(fabs(m.hoist.freeY - freeEnd + 4 * (m.hoist.loadY - load)) < 1e-9);
	load = m.hoist.loadY;
	m.hoist.hold = 0;
	for (int i = 0; i < 120; ++i)
		YMPME_Pulley_Step(&m.hoist, 0, 1.0 / 600);
	CHECK(m.hoist.loadY < load);
	m.hoist.hold = 1;
	for (int i = 0; i < 3000; ++i)
		YMPME_Pulley_Step(&m.hoist, -1, 1.0 / 600);
	CHECK(m.hoist.loadY == -.5);
	/* UI maximum speed and a loaded restart exercise the flexible belt beyond its default speed. */
	YMPME_Belt_Init(&m.belt[1], 1);
	m.belt[1].target = MECH_PI;
	for (int i = 0; i < 3600; ++i)
	{
		YMPME_Belt_Step(&m.belt[1], 1.0 / 600);
		CHECK(isfinite(m.belt[1].omega[1]) && m.belt[1].maxStretch < .04);
	}
	CHECK(m.belt[1].omega[1] > 1.5);
	double before = m.belt[1].omega[1];
	m.belt[1].mu = 0;
	for (int i = 0; i < 3600; ++i)
		YMPME_Belt_Step(&m.belt[1], 1.0 / 600);
	CHECK(m.belt[1].omega[1] < before * .6);

	Swing_Init(&m.swing);
	Swing_Kick(&m.swing);
	CHECK(YMPME_VecDot(m.swing.velocity, m.swing.velocity) > .99);
	CHECK(!Mechanisms_Step(&m, 0, NAN) && !Mechanisms_Step(&m, 0, .1));
	puts("PASS mechanisms: friction transmits/reverses, zero friction cannot drive output; 4:1 hoist; "
		 "endpoint swing and bounded energy");
	return 0;
}
