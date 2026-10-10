#include "YMPME.h"
#include <stdio.h>

int main(void)
{
	/* Steel cantilever, 2 m long, rectangular 20 mm x 40 mm cross section. */
	pme_statics_node nodes[2] = {0};
	pme_statics_element beam = {0};
	pme_statics_model model = {nodes, 2, &beam, 1};
	pme_statics_node_result result[2];
	pme_statics_element_result force;
	double workspace[90];
	pme_statics_status status;
	nodes[0].fixed = PME_STATICS_FIX_X | PME_STATICS_FIX_Y | PME_STATICS_FIX_RZ;
	nodes[1].x = 2;
	nodes[1].load[1] = -10;
	beam.kind = PME_STATICS_BEAM;
	beam.node_b = 1;
	beam.young_modulus = 200e9;
	beam.area = 0.02 * 0.04;
	beam.second_moment = 0.02 * 0.04 * 0.04 * 0.04 / 12;
	status = pme_statics_solve(&model, result, 2, &force, 1, workspace, 90);
	if (status != PME_STATICS_OK)
	{
		fprintf(stderr, "Statics failed: %d\n", (int)status);
		return 1;
	}
	printf("2D linear static analysis: steel cantilever, L=2 m, tip Fy=-10 N\n");
	printf("Tip displacement: %.6f mm, rotation: %.9f rad\n", 1000 * result[1].displacement[1],
		   result[1].displacement[2]);
	printf("Support: Fx=%.6f N, Fy=%.6f N, Mz=%.6f N m\n", result[0].reaction[0],
		   result[0].reaction[1], result[0].reaction[2]);
	printf("Beam end moments: a=%.6f N m, b=%.6f N m\n", force.end_force[2], force.end_force[5]);
	return 0;
}
