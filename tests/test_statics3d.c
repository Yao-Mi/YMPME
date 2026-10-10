#include "STATICS/YMPME_Statics3D.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#define CHECK(c)                                                               \
  do {                                                                         \
    if (!(c)) {                                                                \
      fprintf(stderr, "line %d: %s\n", __LINE__, #c);                          \
      return 1;                                                                \
    }                                                                          \
  } while (0)
#define NEAR(a, b, t) CHECK(fabs((a) - (b)) < (t))
static pme_statics3d_element beam(void) {
  pme_statics3d_element e = {0};
  e.kind = PME_STATICS_BEAM;
  e.node_b = 1;
  e.young_modulus = 200e9;
  e.area = .0008;
  e.shear_modulus = 80e9;
  e.torsion_constant = 3e-8;
  e.second_moment_y = 2e-8;
  e.second_moment_z = 8e-8;
  e.local_y[1] = 1;
  return e;
}
static int cantilever(int rotated) {
  pme_statics3d_node n[2] = {0};
  pme_statics3d_element e = beam();
  pme_statics3d_model m = {n, 2, &e, 1};
  pme_statics3d_node_result r[2];
  pme_statics3d_element_result f;
  double w[324], axes[9], L = 2;
  double loads[6] = {12, -10, 7, 3, 4, -2};
  double expected[6] = {0};
  n[0].fixed = 63;
  if (rotated) {
    n[1].position[0] = 2 / sqrt(3.);
    n[1].position[1] = 2 / sqrt(3.);
    n[1].position[2] = 2 / sqrt(3.);
  } else
    n[1].position[0] = 2;
  CHECK(pme_statics3d_element_frame(&m, &e, axes) == PME_STATICS_OK);
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) {
      n[1].load[i] += axes[j * 3 + i] * loads[j];
      n[1].load[3 + i] += axes[j * 3 + i] * loads[3 + j];
    }
  CHECK(pme_statics3d_solve(&m, r, 2, &f, 1, w, 324) == PME_STATICS_OK);
  expected[0] = loads[0] * L / (e.young_modulus * e.area);
  expected[1] = (loads[1] * L * L * L / 3 + loads[5] * L * L / 2) /
                (e.young_modulus * e.second_moment_z);
  expected[2] = (loads[2] * L * L * L / 3 - loads[4] * L * L / 2) /
                (e.young_modulus * e.second_moment_y);
  expected[3] = loads[3] * L / (e.shear_modulus * e.torsion_constant);
  expected[4] = (-loads[2] * L * L / 2 + loads[4] * L) /
                (e.young_modulus * e.second_moment_y);
  expected[5] = (loads[1] * L * L / 2 + loads[5] * L) /
                (e.young_modulus * e.second_moment_z);
  for (int i = 0; i < 6; ++i) {
    double actual = 0;
    for (int j = 0; j < 3; ++j)
      actual += axes[(i % 3) * 3 + j] * r[1].displacement[(i / 3) * 3 + j];
    NEAR(actual, expected[i], 1e-10);
    NEAR(r[0].displacement[i], 0, 1e-15);
  }
  for (int i = 0; i < 3; ++i) {
    int j = (i + 1) % 3, k = (i + 2) % 3;
    NEAR(r[0].reaction[i] + n[1].load[i], 0, 1e-7);
    NEAR(r[0].reaction[3 + i] + n[1].load[3 + i] +
             n[1].position[j] * n[1].load[k] - n[1].position[k] * n[1].load[j],
         0, 1e-7);
  }
  NEAR(f.axial_force, 12, 1e-7);
  NEAR(f.end_force[9], 3, 1e-7);
  return 0;
}
static int section_roll_and_torsion(void) {
  pme_statics3d_node n[2] = {0};
  pme_statics3d_element e = beam();
  pme_statics3d_model m = {n, 2, &e, 1};
  pme_statics3d_node_result r[2];
  pme_statics3d_element_result f;
  double w[324];
  n[0].fixed = 63;
  n[1].position[0] = 2;
  n[1].load[1] = -10;
  e.local_y[1] = 0;
  e.local_y[2] = 1;
  CHECK(pme_statics3d_solve(&m, r, 2, &f, 1, w, 324) == 0);
  NEAR(r[1].displacement[1],
       -10 * 8 / (3 * e.young_modulus * e.second_moment_y), 1e-10);
  n[1].load[1] = 0;
  n[1].load[3] = 3;
  CHECK(pme_statics3d_solve(&m, r, 2, &f, 1, w, 324) == 0);
  NEAR(r[1].displacement[3], 6 / (e.shear_modulus * e.torsion_constant), 1e-10);
  for (int i = 0; i < 3; ++i)
    NEAR(r[1].displacement[i], 0, 1e-15);
  return 0;
}
static int space_truss(void) {
  pme_statics3d_node n[5] = {0};
  pme_statics3d_element e[4] = {0};
  pme_statics3d_model m = {n, 5, e, 4};
  pme_statics3d_node_result r[5];
  pme_statics3d_element_result f[4];
  double w[1890];
  for (int i = 0; i < 4; ++i) {
    n[i].position[0] = (i & 1) ? 1 : -1;
    n[i].position[2] = (i & 2) ? 1 : -1;
    n[i].fixed = 7;
    e[i].kind = PME_STATICS_TRUSS;
    e[i].node_a = (size_t)i;
    e[i].node_b = 4;
    e[i].young_modulus = 200e9;
    e[i].area = 1e-6;
  }
  n[4].position[1] = 1;
  n[4].load[1] = -10;
  CHECK(pme_statics3d_solve(&m, r, 5, f, 4, w, 1890) == 0);
  NEAR(r[4].displacement[1], -10 * 3 * sqrt(3.) / (4 * 200e9 * 1e-6), 1e-12);
  NEAR(r[4].displacement[0], 0, 1e-12);
  NEAR(r[4].displacement[2], 0, 1e-12);
  for (int i = 0; i < 4; ++i) {
    NEAR(r[i].reaction[1], 2.5, 1e-9);
    NEAR(f[i].axial_force, -2.5 * sqrt(3.), 1e-9);
  }
  /* Horizontal load excites the third dimension, still a stable pyramid. */
  n[4].load[2] = 5;
  CHECK(pme_statics3d_solve(&m, r, 5, f, 4, w, 1890) == 0);
  NEAR(r[4].displacement[2], 5 * 3 * sqrt(3.) / (4 * 200e9 * 1e-6), 1e-12);
  return 0;
}
static int prescribed_and_validation(void) {
  pme_statics3d_node n[2] = {0};
  pme_statics3d_element e = beam();
  pme_statics3d_model m = {n, 2, &e, 1};
  pme_statics3d_node_result r[2];
  pme_statics3d_element_result f;
  double w[324], axes[9];
  CHECK(pme_statics3d_workspace_doubles(2) == 324);
  CHECK(pme_statics3d_workspace_doubles(SIZE_MAX) == 0);
  CHECK(pme_statics3d_workspace_doubles(0) == 0);
  n[0].fixed = 63;
  n[1].position[0] = 2;
  n[0].prescribed[2] = .01;
  n[0].prescribed[4] = .001;
  CHECK(pme_statics3d_solve(&m, r, 2, &f, 1, w, 324) == 0);
  NEAR(r[1].displacement[2], .008, 1e-12);
  NEAR(r[1].displacement[4], .001, 1e-12);
  for (int i = 0; i < 12; ++i)
    NEAR(f.end_force[i], 0, 1e-7);
  CHECK(pme_statics3d_solve(&m, r, 2, &f, 1, w, 323) ==
        PME_STATICS_CAPACITY_EXCEEDED);
  CHECK(pme_statics3d_solve(NULL, r, 2, &f, 1, w, 324) ==
        PME_STATICS_INVALID_ARGUMENT);
  e.local_y[0] = 1;
  e.local_y[1] = 0;
  CHECK(pme_statics3d_element_frame(&m, &e, axes) ==
        PME_STATICS_INVALID_ARGUMENT);
  e = beam();
  e.shear_modulus = 0;
  CHECK(pme_statics3d_solve(&m, r, 2, &f, 1, w, 324) ==
        PME_STATICS_INVALID_ARGUMENT);
  e = beam();
  n[1].position[2] = NAN;
  CHECK(pme_statics3d_solve(&m, r, 2, &f, 1, w, 324) ==
        PME_STATICS_INVALID_ARGUMENT);
  n[1].position[2] = 0;
  memset(n[0].prescribed, 0, sizeof n[0].prescribed);
  n[0].fixed = 0;
  CHECK(pme_statics3d_solve(&m, r, 2, &f, 1, w, 324) == PME_STATICS_SINGULAR);
  n[0].fixed = 7;
  e.kind = PME_STATICS_SPRING;
  e.stiffness = 1000;
  n[1].fixed = 6;
  n[1].load[0] = 10;
  CHECK(pme_statics3d_solve(&m, r, 2, &f, 1, w, 324) == 0);
  NEAR(r[1].displacement[0], .01, 1e-12);
  n[1].load[4] = 1;
  CHECK(pme_statics3d_solve(&m, r, 2, &f, 1, w, 324) == PME_STATICS_SINGULAR);
  return 0;
}
int main(void) {
  CHECK(!cantilever(0));
  CHECK(!cantilever(1));
  CHECK(!section_roll_and_torsion());
  CHECK(!space_truss());
  CHECK(!prescribed_and_validation());
  puts("PASS 3D statics: biaxial bend, torsion, arbitrary orientation, section "
       "roll, pyramid, settlement, validation");
  return 0;
}
