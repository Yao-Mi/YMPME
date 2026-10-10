#include "scene.h"
#include <stdio.h>
static SolidScene s;
int main(void){if(!SolidScene_Build(&s,2,1,0,1000)||!SolidScene_Solve(&s))return 1;printf("Solid block: %zu nodes, %zu TET4 elements\n",s.nc,s.ec);printf("Max displacement %.6f mm, von Mises %.6f MPa, energy %.9f J\n",s.max_displacement*1000,s.max_stress/1e6,s.report.strain_energy);printf("PCG iterations %zu, relative residual %.3g\n",s.report.iterations,s.report.relative_residual);return 0;}
