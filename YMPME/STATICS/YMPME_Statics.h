#ifndef YMPME_STATICS_H
#define YMPME_STATICS_H

#include <stddef.h>

#ifdef __cplusplus
extern "C"
{
#endif

	/* SI units. XY plane; rotations/moments positive counterclockwise about +Z. */
	enum
	{
		PME_STATICS_FIX_X = 1,
		PME_STATICS_FIX_Y = 2,
		PME_STATICS_FIX_RZ = 4
	};
	typedef enum pme_statics_status_
	{
		PME_STATICS_OK = 0,
		PME_STATICS_INVALID_ARGUMENT = -1,
		PME_STATICS_CAPACITY_EXCEEDED = -2,
		PME_STATICS_SINGULAR = -3,
		PME_STATICS_NUMERIC_FAILURE = -4
	} pme_statics_status;

	typedef enum pme_statics_element_kind_
	{
		PME_STATICS_SPRING = 0,
		PME_STATICS_TRUSS = 1,
		PME_STATICS_BEAM = 2
	} pme_statics_element_kind;

	typedef struct pme_statics_node_
	{
		double x, y;
		unsigned fixed;		  /* Bit mask above. Unset prescribed entries must be zero. */
		double prescribed[3]; /* ux [m], uy [m], rz [rad] at fixed DOFs. */
		double load[3];		  /* Fx [N], Fy [N], Mz [N m]. */
	} pme_statics_node;

	typedef struct pme_statics_element_
	{
		pme_statics_element_kind kind;
		size_t node_a, node_b;
		double stiffness;						   /* Spring only: axial k [N/m]. */
		double young_modulus, area, second_moment; /* E [Pa], A [m^2], Iz [m^4]. */
	} pme_statics_element;

	typedef struct pme_statics_model_
	{
		const pme_statics_node *nodes;
		size_t node_count;
		const pme_statics_element *elements;
		size_t element_count;
	} pme_statics_model;

	typedef struct pme_statics_node_result_
	{
		double displacement[3]; /* ux, uy, rz. */
		double reaction[3];		/* K*u - f; free DOFs contain numerical residuals. */
	} pme_statics_node_result;

	typedef struct pme_statics_element_result_
	{
		/* Local +x points a->b; local +y is its CCW normal. K_local*u_local:
		 * [Fx_a, Fy_a, Mz_a, Fx_b, Fy_b, Mz_b], forces applied TO the element. */
		double end_force[6];
		double axial_force;	 /* Positive in tension. */
		double axial_stress; /* N/A for truss/beam; zero for spring. No bending stress. */
	} pme_statics_element_result;

	/* Number of doubles required. Returns zero for zero nodes or size overflow.
	 * Dense O(nodes^2) storage / O(nodes^3) solve; caller owns all storage. */
	size_t pme_statics_workspace_doubles(size_t node_count);

	/* Linear, small-displacement static solve with nodal loads only. No heap use.
	 * Buffers must not overlap. Results are valid only on PME_STATICS_OK.
	 * Unused rotations (nodes with no beam) are automatically zero if unloaded.
	 * Unrestrained translations and unsupported moments return SINGULAR.
	 * SINGULAR also covers numerically ill-conditioned mechanisms. */
	pme_statics_status pme_statics_solve(const pme_statics_model *model,
										 pme_statics_node_result *nodes, size_t node_capacity,
										 pme_statics_element_result *elements,
										 size_t element_capacity, double *workspace,
										 size_t workspace_doubles);

#ifdef __cplusplus
}
#endif
#endif
