#include "soft_finger.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define CHECK(x)                                                                                   \
	do                                                                                             \
	{                                                                                              \
		if (!(x))                                                                                  \
		{                                                                                          \
			fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #x);                                   \
			return 1;                                                                              \
		}                                                                                          \
	} while (0)
int main(int argc, char **argv)
{
	static soft_finger s;
	int ballOnly = argc == 2 && !strcmp(argv[1], "--ball-only");
	int shapes = ballOnly ? 1 : 3;
	float heldCompression[3][2] = {{0}};
	for (int shape = 0; shape < shapes; ++shape)
		for (int hard = 0; hard < 2; ++hard)
		{
			CHECK(SoftFinger_Init(&s, shape, hard));
			float peak = 0, lift = 0;
			size_t contacts = 0;
			for (int i = 0; i < 240 * 10; ++i)
			{
				CHECK(SoftFinger_Step(&s));
				float q = fmaxf(s.result[0].maxCompression, s.result[1].maxCompression);
				peak = fmaxf(peak, q);
				contacts += s.result[0].contacts + s.result[1].contacts;
				if (i > 240 * 5 && i < 240 * 7)
					lift = fmaxf(lift, s.grasp.bodies[3].position.y);
				if (i == 240 * 6)
				{
					heldCompression[shape][hard] = q;
					if (shape == 0)
					{
						pme_vec3 scale = YMPME_SoftBall_Scale(&s.ball);
						CHECK(scale.x < .7f && scale.y > 1.2f);
						CHECK(fabsf(scale.x * scale.y * scale.z - 1) < 2e-6f);
						CHECK(s.grasp.colliders[2].type == PME_COLLIDER_MESH);
					}
					CHECK(s.result[0].bottomedOut == 0 && s.result[1].bottomedOut == 0);
					CHECK(fabsf(s.result[0].other.force.y + s.result[1].other.force.y - 2.943f) <
						  .05f);
				}
				if (i == 240 * 6 && shape == 0)
					printf("BALL q=%.4f scaleX=%.3f Q=%.3f\n", s.ball.compression,
						   YMPME_SoftBall_Scale(&s.ball).x, s.ballForce);
				if (i % 240 == 0)
					printf("shape%d hard%d t%.1f y%.3f q%.2fmm F%.2f %.2f Fy%.2f\n", shape, hard,
						   s.grasp.world.time, s.grasp.bodies[3].position.y, q * 1000,
						   Grasp_NormalForce(&s.grasp, 0), Grasp_NormalForce(&s.grasp, 1),
						   s.grasp.gripForce.y);
			}
			CHECK(contacts > 0);
			CHECK(peak > .0001f);
			CHECK(lift > .65f);
			CHECK(s.result[0].maxCompression < 1e-5f && s.result[1].maxCompression < 1e-5f);
			CHECK(s.grasp.bodies[3].position.y < .25f);
		}
	for (int shape = 0; shape < shapes; ++shape)
		CHECK(heldCompression[shape][1] < heldCompression[shape][0] * .75f);
	/* A firm ball must remain visibly rounder at the same motor effort. */
	CHECK(SoftFinger_Init(&s, 0, 0));
	s.ball.stiffness = 600;
	for (int i = 0; i < 240 * 6; ++i)
		CHECK(SoftFinger_Step(&s));
	CHECK(YMPME_SoftBall_Scale(&s.ball).x > .9f);
	CHECK(s.grasp.bodies[3].position.y > .75f);
	if (ballOnly)
	{
		puts("SOFT BALL GRIP PASS");
		return 0;
	}
	/* A flat-sided object cannot be lifted by frictionless normal pad forces. */
	CHECK(SoftFinger_Init(&s, 1, 0));
	s.grasp.friction = 0;
	for (int i = 0; i < 240 * 6; ++i)
		CHECK(SoftFinger_Step(&s));
	CHECK(s.grasp.bodies[3].position.y < .25f);
	puts("SOFT_FINGER PASS (three shapes, two materials, lift, release)");
	return 0;
}
