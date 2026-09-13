#include "SOFTBODY/YMPME_SoftPad.h"
#include "CORE/YMPME_MathBase.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define V YMPME_Vec3
#define CHECK(x)                                                                                   \
	do                                                                                             \
	{                                                                                              \
		if (!(x))                                                                                  \
		{                                                                                          \
			fprintf(stderr, "line %d: %s\n", __LINE__, #x);                                        \
			exit(1);                                                                               \
		}                                                                                          \
	} while (0)
int main(void)
{
	pme_soft_pad_desc d = YMPME_SoftPad_Default();
	pme_surface_patch p[2] = {{{0, 0, 0}, {1, 0, 0}, {0, 0, 0}, .001f},
							  {{0, .1f, 0}, {1, 0, 0}, {0, 0, 0}, .001f}};
	pme_soft_pad_sample hit[2] = {{1, .005f, {-1, 0, 0}, {0, 0, 0}}, {0}};
	pme_soft_pad_frame frame = {{0, .2f, 0}, {0, 0, 0}, {1, 0, 0}};
	pme_soft_pad_node nodes[2] = {{0}};
	pme_soft_pad_result result;
	for (int i = 0; i < 100; ++i)
		CHECK(YMPME_SoftPad_Step(&d, p, hit, 2, &frame, .004f, nodes, 2, &result) == PME_OK);
	CHECK(fabsf(nodes[0].compression - .005f) < 1e-7f && nodes[1].compression == 0);
	CHECK(fabsf(result.support.force.x + .4f) < 1e-5f && result.contacts == 1);
	CHECK(YMPME_VecDot(YMPME_VecAdd(result.support.force, result.other.force),
					   YMPME_VecAdd(result.support.force, result.other.force)) < 1e-10f);
	pme_vec3 torque = YMPME_VecAdd(
		YMPME_VecAdd(result.support.torque,
					 YMPME_VecCross(frame.supportOrigin, result.support.force)),
		YMPME_VecAdd(result.other.torque, YMPME_VecCross(frame.otherOrigin, result.other.force)));
	CHECK(YMPME_VecDot(torque, torque) < 1e-10f);
	float energy = result.elasticEnergy, compression = nodes[0].compression;
	CHECK(YMPME_SoftPad_Step(&d, p, NULL, 2, &frame, .004f, nodes, 2, &result) == PME_OK);
	CHECK(nodes[0].compression > 0 && nodes[0].compression < compression &&
		  result.elasticEnergy < energy && result.contacts == 0);
	for (int i = 0; i < 100; ++i)
		CHECK(YMPME_SoftPad_Step(&d, p, NULL, 2, &frame, .004f, nodes, 2, &result) == PME_OK);
	CHECK(nodes[0].compression < 1e-7f);
	hit[0].velocity = V(0, 2, 0);
	CHECK(YMPME_SoftPad_Step(&d, p, hit, 2, &frame, .004f, nodes, 2, &result) == PME_OK);
	CHECK(fabsf(nodes[0].force.y) <= d.friction * fabsf(nodes[0].force.x) + 1e-6f);
	/* Disabling friction clears tangential memory and force. */
	d.friction = 0;
	CHECK(YMPME_SoftPad_Step(&d, p, hit, 2, &frame, .004f, nodes, 2, &result) == PME_OK);
	CHECK(nodes[0].force.y == 0 && nodes[0].force.z == 0);
	CHECK(YMPME_VecDot(nodes[0].shear, nodes[0].shear) == 0);
	d.friction = .85f;
	/* A withdrawing obstacle cannot pull a still-compressed pad outward. */
	hit[0].compression = .00001f;
	CHECK(YMPME_SoftPad_Step(&d, p, hit, 2, &frame, .004f, nodes, 2, &result) == PME_OK);
	CHECK(nodes[0].compression > hit[0].compression && !nodes[0].contact);
	CHECK(YMPME_VecDot(nodes[0].force, nodes[0].force) == 0);
	hit[0].compression = .005f;
	pme_soft_pad_node old[2];
	memcpy(old, nodes, sizeof old);
	pme_soft_pad_result saved = result;
	hit[1] = (pme_soft_pad_sample){1, .005f, {NAN, 0, 0}, {0, 0, 0}};
	CHECK(YMPME_SoftPad_Step(&d, p, hit, 2, &frame, .004f, nodes, 2, &result) ==
		  PME_INVALID_ARGUMENT);
	CHECK(!memcmp(old, nodes, sizeof old) && !memcmp(&saved, &result, sizeof result));
	CHECK(YMPME_SoftPad_Step(&d, p, NULL, 2, &frame, .004f, nodes, 1, &result) ==
		  PME_CAPACITY_EXCEEDED);
	CHECK(!memcmp(old, nodes, sizeof old));
	hit[1].hit = 0;
	hit[0].compression = .03f;
	CHECK(YMPME_SoftPad_Step(&d, p, hit, 2, &frame, .004f, nodes, 2, &result) == PME_OK);
	CHECK(result.bottomedOut == 1 && nodes[0].compression == d.maxCompression);
	puts("SOFT PAD PASS: local indentation, free recovery, shear limit, wrench conservation, "
		 "validation");
	return 0;
}
