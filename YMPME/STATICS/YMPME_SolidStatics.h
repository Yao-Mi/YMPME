#ifndef YMPME_SOLID_STATICS_H
#define YMPME_SOLID_STATICS_H
#include <stddef.h>
#ifdef __cplusplus
extern "C" {
#endif
/* Linear isotropic small-strain tetrahedral solid; SI units, right-handed XYZ.
 */
typedef enum {
  PME_SOLID_OK = 0,
  PME_SOLID_INVALID = -1,
  PME_SOLID_CAPACITY = -2,
  PME_SOLID_UNRESTRAINED = -3,
  PME_SOLID_NUMERIC = -4,
  PME_SOLID_NO_CONVERGENCE = -5
} pme_solid_status;
enum {
  PME_SOLID_FIX_X = 1,
  PME_SOLID_FIX_Y = 2,
  PME_SOLID_FIX_Z = 4,
  PME_SOLID_FIX_ALL = 7
};
typedef struct {
  double position[3], load[3], prescribed[3];
  unsigned fixed;
} pme_solid_node;
typedef struct {
  size_t node[4];
} pme_solid_tet;
typedef struct {
  const pme_solid_node *nodes;
  size_t node_count;
  const pme_solid_tet *tets;
  size_t tet_count;
  double young_modulus,
      poisson_ratio; /* -1 < nu < 0.49; nearly incompressible excluded. */
} pme_solid_model;
typedef struct {
  double displacement[3], reaction[3];
} pme_solid_node_result;
typedef struct {
  double strain[6], stress[6]; /* xx,yy,zz,xy,yz,zx; strain uses tensor shear
                                  (not engineering gamma). */
  double von_mises, energy_density,
      volume; /* Pa, J/m^3, m^3; constant in each TET4. */
} pme_solid_tet_result;
typedef struct {
  size_t max_iterations;
  double relative_tolerance, absolute_tolerance;
} pme_solid_options;
typedef struct {
  size_t iterations;
  double residual_norm, relative_residual, strain_energy;
} pme_solid_report;
/* Matrix-free PCG, diagonal preconditioner. No heap; O(nodes+tets) caller
 * workspace. One connected conforming solid, positive-volume nondegenerate tets
 * (either orientation). Constraints must remove mechanisms; unsupported global
 * rigid modes are rejected. Buffers must not overlap. Results only valid on OK;
 * report valid for convergence failure. No contact, plasticity, buckling,
 * inertia or automatic gravity. No stress smoothing. */
size_t pme_solid_workspace_doubles(size_t nodes, size_t tets);
pme_solid_options pme_solid_default_options(void);
pme_solid_status
pme_solid_solve(const pme_solid_model *model, pme_solid_node_result *nodes,
                size_t node_capacity, pme_solid_tet_result *tets,
                size_t tet_capacity, const pme_solid_options *options,
                pme_solid_report *report, double *workspace,
                size_t workspace_doubles);
#ifdef __cplusplus
}
#endif
#endif
