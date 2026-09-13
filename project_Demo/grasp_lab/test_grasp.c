#include "grasp.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define CHECK(x)                                                                                   \
	do                                                                                             \
	{                                                                                              \
		if (!(x))                                                                                  \
		{                                                                                          \
			fprintf(stderr, "line %d: %s\n", __LINE__, #x);                                        \
			exit(1);                                                                               \
		}                                                                                          \
	} while (0)
static float norm(pme_vec3 v) { return sqrtf(YMPME_VecDot(v, v)); }
static int warm, jointWarm;
static pme_joint_cache jointCache;
static pme_joint_cache_entry jointEntries[3];
static pme_contact_cache cache;
static pme_contact_cache_entry entries[32];
static int initShape(grasp_lab *s,int mode,int shape) {
 if(!Grasp_InitShape(s,mode,shape))return 0;
 if(warm){if(YMPME_Dynamics_CacheInit(&cache,entries,32)!=PME_OK)return 0;s->world.cache=&cache;}
 if(jointWarm){if(YMPME_Joint_CacheInit(&jointCache,jointEntries,3)!=PME_OK)return 0;s->world.jointCache=&jointCache;}
 return 1;
}
static int init(grasp_lab *s,int mode){return initShape(s,mode,0);}
int main(int argc, char **argv)
{
	static grasp_lab s;
	jointWarm=argc==2&&!strcmp(argv[1],"--warm-all");
 warm=jointWarm||(argc==2&&!strcmp(argv[1],"--warm-start"));
    int trace = argc > 1 && !warm;
    printf("contact cache: %s, joint cache: %s\n",warm?"ON":"OFF",jointWarm?"ON":"OFF");
	(void)argv;
	CHECK(init(&s, 0));
	float height = 0, closedHeight = 0;
	int both = 0;
	for (int i = 0; i < 2400; ++i)
	{
		CHECK(Grasp_Step(&s));
		if (trace && i % 120 == 0)
			printf("t %.2f ball(%.3f %.3f) omega %.3f | angle %.3f %.3f | F %.3f %.3f Fy %.3f "
				   "ground %.3f\n",
				   s.world.time, s.bodies[3].position.x, s.bodies[3].position.y,
				   s.bodies[3].angularVelocity.z, s.joints[1].coordinate, s.joints[2].coordinate,
				   Grasp_NormalForce(&s, 0), Grasp_NormalForce(&s, 1), s.gripForce.y,
				   s.groundForce.y);
		if (i == 480)
			closedHeight = s.bodies[3].position.y;
		if (i == 1440)
			height = s.bodies[3].position.y;
		if (i > 720 && i < 1440 && Grasp_NormalForce(&s, 0) > .1f && Grasp_NormalForce(&s, 1) > .1f)
			++both;
		CHECK(norm(YMPME_VecAdd(YMPME_VecAdd(s.fingerForce[0], s.fingerForce[1]), s.gripForce)) <
			  1e-5f);
		for (int side = 0; side < 2; ++side)
			CHECK(norm(YMPME_VecSub(s.tactile[side].wrench.force, s.fingerForce[side])) < 1e-5f);
		CHECK(norm(YMPME_VecSub(s.objectContact.force, YMPME_VecAdd(s.gripForce, s.groundForce))) <
			  1e-4f);
		CHECK(s.joints[1].feedback.anchorError < .01f && s.joints[2].feedback.anchorError < .01f);
	}
	printf("grip start=%.3f lifted=%.3f release=%.3f both=%d\n", closedHeight, height,
		   s.bodies[3].position.y, both);
	CHECK(height > closedHeight + .35f);
	CHECK(both > 500);
	CHECK(s.bodies[3].position.y < .24f);
	CHECK(init(&s, 0));
	s.friction = 0;
	for (int i = 0; i < 1440; ++i)
		CHECK(Grasp_Step(&s));
	printf("frictionless height %.3f\n", s.bodies[3].position.y);
	CHECK(s.bodies[3].position.y < height - .25f);
	CHECK(init(&s, 1));
	float omega = 0, x = 0;
	for (int i = 0; i < 720; ++i)
	{
		CHECK(Grasp_Step(&s));
		omega = fmaxf(omega, fabsf(s.bodies[3].angularVelocity.z));
		x = fmaxf(x, s.bodies[3].position.x);
	}
	printf("push x %.3f max spin %.3f\n", x, omega);
	CHECK(x > .2f);
	CHECK(omega > .5f);
	for (int shape = 1; shape <= 2; ++shape)
	{
		CHECK(initShape(&s, 0, shape));
		float lifted = 0;
		int gripping = 0;
		for (int i = 0; i < 2400; ++i)
		{
			CHECK(Grasp_Step(&s));
			if (i == 1440)
				lifted = s.bodies[3].position.y;
			if (i > 720 && i < 1440 && Grasp_NormalForce(&s, 0) > .1f &&
				Grasp_NormalForce(&s, 1) > .1f)
				++gripping;
			CHECK(norm(YMPME_VecAdd(YMPME_VecAdd(s.fingerForce[0], s.fingerForce[1]),
									s.gripForce)) < 1e-4f);
			for (int finger = 0; finger < 2; ++finger)
				CHECK(norm(YMPME_VecSub(s.tactile[finger].wrench.force, s.fingerForce[finger])) <
					  1e-4f);
			if (trace && i % 240 == 0)
				printf("shape %d t %.2f y %.3f Fy %.3f contacts %zu\n", shape, s.world.time,
					   s.bodies[3].position.y, s.gripForce.y, s.world.contactCount);
		}
		printf("shape %d lift %.3f release %.3f gripping %d\n", shape, lifted,
			   s.bodies[3].position.y, gripping);
		CHECK(lifted > .55f && gripping > 500 && s.bodies[3].position.y < .25f);
		CHECK(initShape(&s, 0, shape));
		s.friction = 0;
		for (int i = 0; i < 1440; ++i)
			CHECK(Grasp_Step(&s));
		printf("shape %d frictionless %.3f\n", shape, s.bodies[3].position.y);
		CHECK(s.bodies[3].position.y < lifted - .25f);
	}
	CHECK(Grasp_CellCount(2) == 0);
	puts("GRASP PASS: lift, release, friction necessity, rolling, bilateral tactile conservation");
	return 0;
}
