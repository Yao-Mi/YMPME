#include "STATICS/YMPME_SolidStatics.h"
#include "scene.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#define CHECK(x)                                                               \
  do {                                                                         \
    if (!(x)) {                                                                \
      fprintf(stderr, "line %d: %s\n", __LINE__, #x);                          \
      return 1;                                                                \
    }                                                                          \
  } while (0)
#define NEAR(a, b, t) CHECK(fabs((a) - (b)) < (t))
static SolidScene s;
static int affine(void) {
  CHECK(SolidScene_Build(&s, 1, 0, 0, 1000));
  double grad[3][3] = {
      {.001, .002, -.0004}, {-.0005, -.0003, .0006}, {.0009, -.0002, .0002}};
  double expected[6] = {.001, -.0003, .0002, .00075, .0002, .00025}, stress[6],
         mu = 70e9 / 2.6, lambda = 70e9 * .3 / (1.3 * .4);
  for (int i = 0; i < 6; ++i)
    stress[i] = 2 * mu * expected[i] + (i < 3 ? lambda * .0009 : 0);
  for (size_t i = 0; i < s.nc; ++i) {
    pme_solid_node *n = &s.nodes[i];
    int boundary = 0;
    for (int j = 0; j < 3; ++j)
      if (n->position[j] < 1e-10 || fabs(n->position[j] - .2) < 1e-10)
        boundary = 1;
    n->fixed = boundary ? 7 : 0;
    for (int j = 0; j < 3; ++j) {
      n->load[j] = 0;
      n->prescribed[j] = 0;
      if (boundary)
        for (int k = 0; k < 3; ++k)
          n->prescribed[j] += grad[j][k] * n->position[k];
    }
  }
  CHECK(SolidScene_Solve(&s));
  for (size_t e = 0; e < s.ec; ++e)
    for (int j = 0; j < 6; ++j) {
      NEAR(s.er[e].strain[j], expected[j], 1e-9);
      NEAR(s.er[e].stress[j], stress[j], 100);
    }
  for (size_t i = 0; i < s.nc; ++i)
    for (int j = 0; j < 3; ++j) {
      double u = 0;
      for (int k = 0; k < 3; ++k)
        u += grad[j][k] * s.nodes[i].position[k];
      NEAR(s.nr[i].displacement[j], u, 1e-10);
    }
  return 0;
}
static int uniaxial_and_inversion(void) {
  CHECK(SolidScene_Build(&s, 1, 1, 0, 1000));
  /* Roller planes remove rigid modes while allowing exact free Poisson
   * contraction. */
  for (size_t i = 0; i < s.nc; ++i) {
    s.nodes[i].fixed = 0;
    for (int j = 0; j < 3; ++j)
      if (s.nodes[i].position[j] < 1e-10)
        s.nodes[i].fixed |= 1u << j;
  }
  /* Reversing orientation must not change physical gradients or volume. */
  for (size_t e = 0; e < s.ec; e += 2) {
    size_t tmp = s.tets[e].node[0];
    s.tets[e].node[0] = s.tets[e].node[1];
    s.tets[e].node[1] = tmp;
  }
  CHECK(SolidScene_Solve(&s));
  double sigma = -1000 / .04, energy = 0;
  for (size_t i = 0; i < s.nc; ++i) {
    NEAR(s.nr[i].displacement[0], sigma / 70e9 * s.nodes[i].position[0], 1e-12);
    NEAR(s.nr[i].displacement[1], -.3 * sigma / 70e9 * s.nodes[i].position[1],
         1e-12);
  }
  for (size_t e = 0; e < s.ec; ++e) {
    NEAR(s.er[e].stress[0], sigma, .1);
    NEAR(s.er[e].von_mises, -sigma, .1);
    NEAR(s.er[e].volume, .008 / 3072, 1e-14);
    energy += s.er[e].energy_density * s.er[e].volume;
  }
  NEAR(energy, .5 * sigma * sigma / 70e9 * .008, 1e-9);
  return 0;
}
static int bending_balance(void) {
  CHECK(SolidScene_Build(&s, 2, 0, 0, 1000));
  CHECK(SolidScene_Solve(&s));
  double coarse = s.max_displacement;
  CHECK(SolidScene_Build(&s, 2, 1, 0, 1000));
  CHECK(SolidScene_Solve(&s));
  CHECK(s.max_displacement > coarse);
  double force[3] = {0}, moment[3] = {0}, external_work = 0;
  for (size_t i = 0; i < s.nc; ++i) {
    double f[3];
    for (int j = 0; j < 3; ++j) {
      f[j] = s.nodes[i].load[j] + s.nr[i].reaction[j];
      force[j] += f[j];
      external_work += s.nodes[i].load[j] * s.nr[i].displacement[j];
      if (s.nodes[i].fixed & (1u << j))
        NEAR(s.nr[i].displacement[j], 0, 1e-20);
    }
    for (int j = 0; j < 3; ++j) {
      int a = (j + 1) % 3, b = (j + 2) % 3;
      moment[j] +=
          s.nodes[i].position[a] * f[b] - s.nodes[i].position[b] * f[a];
    }
  }
  for (int j = 0; j < 3; ++j) {
    NEAR(force[j], 0, 1e-5);
    NEAR(moment[j], 0, 1e-5);
  }
  NEAR(external_work, 2 * s.report.strain_energy, 1e-8);
  printf("Bending refinement: u_max %.9g -> %.9g m, PCG %zu iterations, "
         "residual %.3g\n",
         coarse, s.max_displacement, s.report.iterations,
         s.report.relative_residual);
  return 0;
}
static int validation(void) {
  CHECK(SolidScene_Build(&s, 1, 0, 0, 1000));
  pme_solid_model m = {s.nodes, s.nc, s.tets, s.ec, 70e9, .3};
  pme_solid_options o = pme_solid_default_options();
  CHECK(pme_solid_workspace_doubles(0, 1) == 0);
  CHECK(pme_solid_workspace_doubles(SIZE_MAX, 1) == 0);
  CHECK(pme_solid_solve(&m, s.nr, SOLID_NODES, s.er, SOLID_TETS, &o, &s.report,
                        s.workspace, 1) == PME_SOLID_CAPACITY);
  o.max_iterations = 1;
  CHECK(pme_solid_solve(&m, s.nr, SOLID_NODES, s.er, SOLID_TETS, &o, &s.report,
                        s.workspace, SOLID_WORK) == PME_SOLID_NO_CONVERGENCE);
  for (size_t i = 0; i < s.nc; ++i)
    s.nodes[i].fixed = 0;
  CHECK(!SolidScene_Solve(&s));
  CHECK(s.status == PME_SOLID_UNRESTRAINED);
  s.tets[0].node[1] = s.tets[0].node[0];
  CHECK(!SolidScene_Solve(&s));
  CHECK(s.status == PME_SOLID_INVALID);
  return 0;
}
int main(void) {
  CHECK(!affine());
  CHECK(!uniaxial_and_inversion());
  CHECK(!bending_balance());
  CHECK(!validation());
  puts("PASS solid FEM: affine patch, Poisson contraction, stress, energy, "
       "orientation, refinement, reactions, validation");
  return 0;
}
