#include "YMPME.h"
#include <float.h>
#include <stdio.h>
#include <string.h>
#if !PME_ENABLE_FLIGHT
#error "Linking YMPME::flight must expose flight declarations"
#endif
#define CHECK(x)                                                                                             \
	do                                                                                                       \
	{                                                                                                        \
		if (!(x))                                                                                            \
		{                                                                                                    \
			fprintf(stderr, "line %d: %s\n", __LINE__, #x);                                                  \
			return 1;                                                                                        \
		}                                                                                                    \
	} while (0)
#define NEAR(a, b) CHECK(fabsf((a) - (b)) < 1e-4f)
int main(void)
{
	pme_flight_load a = YMPME_Flight_ZeroLoad(), b, c, old;
	NEAR(a.force.x, 0);
	NEAR(a.torque.z, 0);
	CHECK(YMPME_Flight_LoadAtPoint(YMPME_Vec3(0, 3, 0), YMPME_Vec3(2, 0, 0), YMPME_Vec3(0, 1, 0), &a) ==
	      PME_OK);
	NEAR(a.force.y, 3);
	NEAR(a.torque.z, 6);
	NEAR(a.torque.y, 1);
	b = a;
	CHECK(YMPME_Flight_AddLoads(&a, &b, &a) == PME_OK);
	NEAR(a.force.y, 6);
	NEAR(a.torque.z, 12);
	old = a;
	CHECK(YMPME_Flight_LoadAtPoint(YMPME_Vec3(0, FLT_MAX, 0), YMPME_Vec3(2, 0, 0), YMPME_Vec3(0, 0, 0), &a) ==
	      PME_INVALID_ARGUMENT);
	CHECK(!memcmp(&old, &a, sizeof(a)));
	b.force.x = NAN;
	CHECK(YMPME_Flight_AddLoads(&a, &b, &a) == PME_INVALID_ARGUMENT);
	CHECK(!memcmp(&old, &a, sizeof(a)));
	c = YMPME_Flight_ZeroLoad();
	(void)c;
#if PME_FLIGHT_ROTORS
	{
		pme_rotor_desc rotor = {{1, 0, 0}, {0, 2, 0}, .01f, .001f};
		CHECK(YMPME_Flight_RotorLoad(&rotor, 10, &a) == PME_OK);
		NEAR(a.force.y, 1);
		NEAR(a.torque.z, 1);
		NEAR(a.torque.y, -.1f);
		CHECK(YMPME_Flight_RotorLoad(&rotor, -10, &b) == PME_OK);
		NEAR(b.force.y, 1);
		NEAR(b.torque.y, .1f);
		/* 四个对称旋翼，反扭矩和偏置力矩相消。 */
		c = YMPME_Flight_ZeroLoad();
		for (int i = 0; i < 4; ++i)
		{
			rotor.position = YMPME_Vec3(i % 2 ? 1 : -1, 0, i < 2 ? 1 : -1);
			CHECK(YMPME_Flight_RotorLoad(&rotor, (i == 0 || i == 3) ? 10 : -10, &a) == PME_OK);
			CHECK(YMPME_Flight_AddLoads(&c, &a, &c) == PME_OK);
		}
		NEAR(c.force.y, 4);
		NEAR(c.torque.x, 0);
		NEAR(c.torque.y, 0);
		NEAR(c.torque.z, 0);
		old = a;
		rotor.axis = YMPME_Vec3(0, 0, 0);
		CHECK(YMPME_Flight_RotorLoad(&rotor, 10, &a) == PME_INVALID_ARGUMENT);
		CHECK(!memcmp(&old, &a, sizeof(a)));
		rotor.axis.y = 1;
		CHECK(YMPME_Flight_RotorLoad(&rotor, FLT_MAX, &a) == PME_INVALID_ARGUMENT);
		CHECK(!memcmp(&old, &a, sizeof(a)));
		CHECK(YMPME_Flight_RotorLoad(&rotor, 0, &a) == PME_OK);
		NEAR(a.force.y, 0);
		NEAR(a.torque.y, 0);
	}
#endif
#if PME_FLIGHT_AERODYNAMICS
	{
		pme_aero_desc aero = {{1, 0, 0}, {0, 1, 0}, 2, .5f, .1f};
		CHECK(YMPME_Flight_AeroLoad(&aero, YMPME_Vec3(0, 0, 10), 1.2f, &a) == PME_OK);
		NEAR(a.force.y, 60);
		NEAR(a.force.z, -12);
		NEAR(a.torque.y, 12);
		NEAR(a.torque.z, 60);
		CHECK(YMPME_Flight_AeroLoad(&aero, YMPME_Vec3(0, 0, 20), 1.2f, &b) == PME_OK);
		NEAR(b.force.y, 4 * a.force.y);
		CHECK(YMPME_Flight_AeroLoad(&aero, YMPME_Vec3(0, 0, -10), 1.2f, &b) == PME_OK);
		NEAR(b.force.z, 12);
		aero.dragCoefficient = 0;
		CHECK(YMPME_Flight_AeroLoad(&aero, YMPME_Vec3(2, 3, 4), 1, &a) == PME_OK);
		NEAR(YMPME_VecDot(a.force, YMPME_Vec3(2, 3, 4)), 0);
		aero.liftCoefficient = 0;
		aero.dragCoefficient = .1f;
		CHECK(YMPME_Flight_AeroLoad(&aero, YMPME_Vec3(2, 3, 4), 1, &a) == PME_OK);
		CHECK(YMPME_VecDot(a.force, YMPME_Vec3(2, 3, 4)) < 0);
		aero.liftCoefficient = -.5f;
		CHECK(YMPME_Flight_AeroLoad(&aero, YMPME_Vec3(0, 0, 10), 1.2f, &a) == PME_OK);
		NEAR(a.force.y, -60);
		old = a;
		CHECK(YMPME_Flight_AeroLoad(&aero, YMPME_Vec3(0, 10, 0), 1, &a) == PME_INVALID_ARGUMENT);
		CHECK(!memcmp(&a, &old, sizeof(a)));
		CHECK(YMPME_Flight_AeroLoad(&aero, YMPME_Vec3(0, 0, 0), 1, &a) == PME_OK);
		NEAR(a.force.y, 0);
		CHECK(YMPME_Flight_AeroLoad(&aero, YMPME_Vec3(0, 0, 10), 0, &a) == PME_OK);
		NEAR(a.force.z, 0);
		old = a;
		CHECK(YMPME_Flight_AeroLoad(&aero, YMPME_Vec3(0, 0, 10), -1, &a) == PME_INVALID_ARGUMENT);
		CHECK(!memcmp(&a, &old, sizeof(a)));
	}
#endif
	printf("PASS flight loads: rotors=%d aerodynamics=%d, aliasing and invalid-input rollback\n",
	       PME_FLIGHT_ROTORS, PME_FLIGHT_AERODYNAMICS);
	return 0;
}
