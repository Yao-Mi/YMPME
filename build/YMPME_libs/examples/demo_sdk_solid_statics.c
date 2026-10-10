#include "STATICS/YMPME_SolidStatics.h"
#include <math.h>
#include <stdio.h>
int main(void)
{
    pme_solid_node n[4] = {0};
    pme_solid_tet element = {{0, 1, 2, 3}};
    pme_solid_node_result result[4];
    pme_solid_tet_result stress;
    pme_solid_report report;
    double workspace[97];
    n[1].position[0] = 1; n[2].position[1] = 1; n[3].position[2] = 1;
    for (int i = 0; i < 3; ++i) n[i].fixed = PME_SOLID_FIX_ALL;
    n[3].load[2] = -10;
    pme_solid_model model = {n, 4, &element, 1, 1e6, .3};
    if (pme_solid_solve(&model, result, 4, &stress, 1, NULL, &report,
                        workspace, 97) != PME_SOLID_OK) return 1;
    /* One unit TET, fixed base: Kzz=(lambda+2*mu)/6; sigma_zz=F/volume. */
    double expected = -60 / (1e6 * .7 / (1.3 * .4));
    if (fabs(result[3].displacement[2] - expected) > 1e-12 ||
        fabs(stress.stress[2] + 60) > 1e-8) return 1;
    printf("TET4 displacement %.9f mm, sigma_zz %.6f Pa, PCG %zu iterations\n",
           result[3].displacement[2]*1000, stress.stress[2], report.iterations);
    return 0;
}
