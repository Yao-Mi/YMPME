#include "YMPME.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>

#define CHECK(condition)                                                                           \
	do                                                                                             \
	{                                                                                              \
		if (!(condition))                                                                          \
		{                                                                                          \
			fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #condition);                        \
			return 1;                                                                              \
		}                                                                                          \
	} while (0)
#define NEAR(actual, expected, tol) CHECK(fabs((actual) - (expected)) <= (tol))

static int spring_and_settlement(void)
{
	pme_statics_node nodes[2] = {0};
	pme_statics_element element = {PME_STATICS_SPRING, 0, 1, 1000, 0, 0, 0};
	pme_statics_model m = {nodes, 2, &element, 1};
	pme_statics_node_result r[2];
	pme_statics_element_result f;
	double w[90];
	nodes[0].fixed = 3;
	nodes[0].prescribed[0] = 0.02;
	nodes[1].x = 1;
	nodes[1].fixed = PME_STATICS_FIX_Y;
	nodes[1].load[0] = 10;
	CHECK(pme_statics_solve(&m, r, 2, &f, 1, w, 90) == PME_STATICS_OK);
	NEAR(r[1].displacement[0], 0.03, 1e-12);
	NEAR(r[0].reaction[0], -10, 1e-10);
	NEAR(r[1].reaction[0], 0, 1e-10);
	NEAR(f.axial_force, 10, 1e-10);
	NEAR(r[1].displacement[2], 0, 1e-12);
	/* Prescribed extension: reactions must balance with no applied nodal force. */
	nodes[1].fixed = 3;
	nodes[1].prescribed[0] = 0.04;
	nodes[1].load[0] = 0;
	CHECK(pme_statics_solve(&m, r, 2, &f, 1, w, 90) == PME_STATICS_OK);
	NEAR(r[0].reaction[0], -20, 1e-10);
	NEAR(r[1].reaction[0], 20, 1e-10);
	return 0;
}

static int truss(void)
{
	pme_statics_node nodes[3] = {0};
	pme_statics_element e[2] = {{PME_STATICS_TRUSS, 0, 2, 0, 200e9, 1e-4, 0},
								{PME_STATICS_TRUSS, 1, 2, 0, 200e9, 1e-4, 0}};
	pme_statics_model m = {nodes, 3, e, 2};
	pme_statics_node_result r[3];
	pme_statics_element_result f[2];
	double w[189], p = 100, length = sqrt(2.0);
	nodes[0].x = -1;
	nodes[1].x = 1;
	nodes[0].fixed = nodes[1].fixed = 3;
	nodes[2].y = 1;
	nodes[2].load[1] = -p;
	CHECK(pme_statics_solve(&m, r, 3, f, 2, w, 189) == PME_STATICS_OK);
	NEAR(r[2].displacement[0], 0, 1e-12);
	NEAR(r[2].displacement[1], -p * length / (200e9 * 1e-4), 1e-12);
	NEAR(f[0].axial_force, -p / sqrt(2.0), 1e-8);
	NEAR(f[1].axial_stress, -p / sqrt(2.0) / 1e-4, 1e-5);
	NEAR(r[0].reaction[1], p / 2, 1e-8);
	NEAR(r[1].reaction[1], p / 2, 1e-8);
	NEAR(r[0].reaction[0] + r[1].reaction[0], 0, 1e-8);
	return 0;
}

static int cantilever(double c, double s)
{
	pme_statics_node nodes[2] = {0};
	pme_statics_element e = {PME_STATICS_BEAM, 0, 1, 0, 200e9, 8e-4, 1e-7};
	pme_statics_model m = {nodes, 2, &e, 1};
	pme_statics_node_result r[2];
	pme_statics_element_result f;
	double w[90], p = 10, length = 2, ei = e.young_modulus * e.second_moment;
	double deflection = p * length * length * length / (3 * ei);
	nodes[0].fixed = 7;
	nodes[1].x = length * c;
	nodes[1].y = length * s;
	nodes[1].load[0] = p * s;
	nodes[1].load[1] = -p * c;
	CHECK(pme_statics_solve(&m, r, 2, &f, 1, w, 90) == PME_STATICS_OK);
	NEAR(r[1].displacement[0], s * deflection, 1e-11);
	NEAR(r[1].displacement[1], -c * deflection, 1e-11);
	NEAR(r[1].displacement[2], -p * length * length / (2 * ei), 1e-11);
	NEAR(r[0].reaction[0], -p * s, 1e-7);
	NEAR(r[0].reaction[1], p * c, 1e-7);
	NEAR(r[0].reaction[2], p * length, 1e-7);
	NEAR(f.end_force[2], p * length, 1e-7);
	NEAR(f.end_force[5], 0, 1e-7);
	NEAR(f.end_force[1], p, 1e-7);
	/* Pure tip couple. */
	nodes[1].load[0] = nodes[1].load[1] = 0;
	nodes[1].load[2] = 3;
	CHECK(pme_statics_solve(&m, r, 2, &f, 1, w, 90) == PME_STATICS_OK);
	NEAR(r[1].displacement[2], 3 * length / ei, 1e-11);
	NEAR(r[1].displacement[1], c * 3 * length * length / (2 * ei), 1e-11);
	NEAR(r[0].reaction[2], -3, 1e-7);
	/* A prescribed infinitesimal rigid rotation must produce no strain. */
	nodes[1].load[2] = 0;
	nodes[1].fixed = 7;
	nodes[0].prescribed[2] = nodes[1].prescribed[2] = 0.001;
	nodes[1].prescribed[0] = -0.001 * nodes[1].y;
	nodes[1].prescribed[1] = 0.001 * nodes[1].x;
	CHECK(pme_statics_solve(&m, r, 2, &f, 1, w, 90) == PME_STATICS_OK);
	for (size_t i = 0; i < 6; ++i)
		NEAR(f.end_force[i], 0, 1e-7);
	return 0;
}

static int simply_supported(void)
{
	pme_statics_node nodes[3] = {0};
	pme_statics_element e[2] = {{PME_STATICS_BEAM, 0, 1, 0, 200e9, 8e-4, 1e-7},
								{PME_STATICS_BEAM, 1, 2, 0, 200e9, 8e-4, 1e-7}};
	pme_statics_model m = {nodes, 3, e, 2};
	pme_statics_node_result r[3];
	pme_statics_element_result f[2];
	double w[189], p = 10, length = 2;
	nodes[0].fixed = 3;
	nodes[1].x = 1;
	nodes[2].x = 2;
	nodes[2].fixed = PME_STATICS_FIX_Y;
	nodes[1].load[1] = -p;
	CHECK(pme_statics_solve(&m, r, 3, f, 2, w, 189) == PME_STATICS_OK);
	NEAR(r[1].displacement[1], -p * length * length * length / (48 * 200e9 * 1e-7), 1e-12);
	NEAR(r[1].displacement[2], 0, 1e-12);
	NEAR(r[0].reaction[1], p / 2, 1e-9);
	NEAR(r[2].reaction[1], p / 2, 1e-9);
	NEAR(f[0].end_force[2], 0, 1e-9);
	NEAR(f[1].end_force[5], 0, 1e-9);
	NEAR(f[0].end_force[5], p * length / 4, 1e-9);
	NEAR(f[0].end_force[5] + f[1].end_force[2], 0, 1e-9);
	return 0;
}

static int failures(void)
{
	pme_statics_node nodes[2] = {0};
	pme_statics_element e = {PME_STATICS_SPRING, 0, 1, 1000, 0, 0, 0};
	pme_statics_model m = {nodes, 2, &e, 1};
	pme_statics_node_result r[2];
	pme_statics_element_result f;
	double w[90];
	CHECK(pme_statics_workspace_doubles(2) == 90);
	CHECK(pme_statics_workspace_doubles(3) == 189);
	CHECK(pme_statics_workspace_doubles(0) == 0);
	CHECK(pme_statics_workspace_doubles(SIZE_MAX) == 0);
	CHECK(pme_statics_workspace_doubles(SIZE_MAX / 32) == 0);
	CHECK(pme_statics_solve(NULL, r, 2, &f, 1, w, 90) == PME_STATICS_INVALID_ARGUMENT);
	CHECK(pme_statics_solve(&m, r, 2, &f, 1, w, 89) == PME_STATICS_CAPACITY_EXCEEDED);
	CHECK(pme_statics_solve(&m, r, 1, &f, 1, w, 90) == PME_STATICS_CAPACITY_EXCEEDED);
	CHECK(pme_statics_solve(&m, r, 2, &f, 0, w, 90) == PME_STATICS_CAPACITY_EXCEEDED);
	CHECK(pme_statics_solve(&m, r, 2, &f, 1, w, 90) == PME_STATICS_INVALID_ARGUMENT);
	nodes[1].x = 1;
	nodes[0].fixed = nodes[1].fixed = PME_STATICS_FIX_Y;
	/* Nonzero diagonal but free rigid translation: must not silently pin it. */
	CHECK(pme_statics_solve(&m, r, 2, &f, 1, w, 90) == PME_STATICS_SINGULAR);
	nodes[0].fixed = 3;
	nodes[1].fixed = 0;
	CHECK(pme_statics_solve(&m, r, 2, &f, 1, w, 90) == PME_STATICS_SINGULAR);
	nodes[1].fixed = PME_STATICS_FIX_Y;
	nodes[1].load[2] = 1;
	CHECK(pme_statics_solve(&m, r, 2, &f, 1, w, 90) == PME_STATICS_SINGULAR);
	nodes[1].load[2] = 0;
	e.stiffness = -1;
	CHECK(pme_statics_solve(&m, r, 2, &f, 1, w, 90) == PME_STATICS_INVALID_ARGUMENT);
	e.stiffness = 1000;
	nodes[1].load[0] = NAN;
	CHECK(pme_statics_solve(&m, r, 2, &f, 1, w, 90) == PME_STATICS_INVALID_ARGUMENT);
	nodes[1].load[0] = 1;
	e.node_b = 2;
	CHECK(pme_statics_solve(&m, r, 2, &f, 1, w, 90) == PME_STATICS_INVALID_ARGUMENT);
	e.node_b = 1;
	nodes[1].prescribed[0] = 0.01;
	CHECK(pme_statics_solve(&m, r, 2, &f, 1, w, 90) == PME_STATICS_INVALID_ARGUMENT);
	return 0;
}

int main(void)
{
	CHECK(spring_and_settlement() == 0);
	CHECK(truss() == 0);
	CHECK(cantilever(1, 0) == 0);
	CHECK(cantilever(0.6, 0.8) == 0);
	CHECK(cantilever(0, 1) == 0);
	CHECK(simply_supported() == 0);
	CHECK(failures() == 0);
	puts("Statics: analytical displacement, reaction, internal force and failure checks passed.");
	return 0;
}
