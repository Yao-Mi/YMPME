#ifndef YMPME_STATICS3D_H
#define YMPME_STATICS3D_H
#include "YMPME_Statics.h" /* Shared status and element-kind enums; no 2D runtime dependency. */
#ifdef __cplusplus
extern "C" {
#endif
/* Right-handed world coordinates, SI units, right-hand rotations in radians.
 * DOF order: ux, uy, uz, rx, ry, rz. */
enum {
  PME_STATICS3D_FIX_X = 1,
  PME_STATICS3D_FIX_Y = 2,
  PME_STATICS3D_FIX_Z = 4,
  PME_STATICS3D_FIX_RX = 8,
  PME_STATICS3D_FIX_RY = 16,
  PME_STATICS3D_FIX_RZ = 32,
  PME_STATICS3D_FIX_TRANSLATION = 7,
  PME_STATICS3D_FIX_ALL = 63
};
typedef struct {
  double position[3];
  unsigned fixed;
  double prescribed[6]; /* Unconstrained entries must be zero. */
  double load[6];       /* Fx,Fy,Fz,Mx,My,Mz. */
} pme_statics3d_node;
typedef struct {
  pme_statics_element_kind kind;
  size_t node_a, node_b;
  double stiffness; /* Axial spring k. */
  double young_modulus, area;
  double shear_modulus, torsion_constant, second_moment_y, second_moment_z;
  /* Beam only: world-space reference for local +y, projected normal to a->b.
   * Must be finite, nonzero, not parallel to the beam. Local z = x cross y. */
  double local_y[3];
} pme_statics3d_element;
typedef struct {
  const pme_statics3d_node *nodes;
  size_t node_count;
  const pme_statics3d_element *elements;
  size_t element_count;
} pme_statics3d_model;
typedef struct {
  double displacement[6], reaction[6]; /* reaction = K*u-f. */
} pme_statics3d_node_result;
typedef struct {
  /* Local end forces applied TO the element: Fx,Fy,Fz,Mx,My,Mz at a, then b. */
  double end_force[12];
  double axial_force,
      axial_stress; /* Tension positive; N/A only, no bending stress. */
} pme_statics3d_element_result;
/* Caller-owned dense storage: n=6*node_count, 2*n*n+3*n doubles. No heap. */
size_t pme_statics3d_workspace_doubles(size_t node_count);
/* Local x,y,z unit axes in world coordinates, consecutive triples (world->local
 * rows). Usable by external display code without a rendering dependency in the
 * solver. */
pme_statics_status
pme_statics3d_element_frame(const pme_statics3d_model *model,
                            const pme_statics3d_element *element,
                            double axes[9]);
/* Small-displacement linear axial spring/truss and Euler-Bernoulli space beam.
 * Nodal loads only. No shear deformation, contact, releases, buckling or solid
 * FEM. Unused unloaded rotation DOFs auto-zero; free translations/mechanisms
 * are SINGULAR. No aliasing between model, results and workspace. Results valid
 * only on OK. */
pme_statics_status pme_statics3d_solve(
    const pme_statics3d_model *model, pme_statics3d_node_result *nodes,
    size_t node_capacity, pme_statics3d_element_result *elements,
    size_t element_capacity, double *workspace, size_t workspace_doubles);
#ifdef __cplusplus
}
#endif
#endif
