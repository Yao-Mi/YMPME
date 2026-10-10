#include "STATICS/YMPME_Statics3D.h"
#include <stdio.h>
int main(void)
{
    pme_statics3d_node n[2]={0};pme_statics3d_element e={0};
    pme_statics3d_model m={n,2,&e,1};pme_statics3d_node_result r[2];
    pme_statics3d_element_result f;double w[324];
    n[0].fixed=PME_STATICS3D_FIX_ALL;n[1].position[0]=2;
    n[1].load[1]=-10;n[1].load[2]=7;n[1].load[3]=3;
    e.kind=PME_STATICS_BEAM;e.node_b=1;e.young_modulus=200e9;e.area=.0008;
    e.shear_modulus=80e9;e.torsion_constant=3e-8;e.second_moment_y=2e-8;e.second_moment_z=8e-8;e.local_y[1]=1;
    if(pme_statics3d_solve(&m,r,2,&f,1,w,324)!=PME_STATICS_OK)return 1;
    printf("Tip displacement [mm]: %.6f %.6f %.6f\n",1000*r[1].displacement[0],1000*r[1].displacement[1],1000*r[1].displacement[2]);
    printf("Tip rotation [rad]: %.9f %.9f %.9f\n",r[1].displacement[3],r[1].displacement[4],r[1].displacement[5]);
    printf("Support force [N]: %.6f %.6f %.6f\n",r[0].reaction[0],r[0].reaction[1],r[0].reaction[2]);
    printf("Support moment [N m]: %.6f %.6f %.6f\n",r[0].reaction[3],r[0].reaction[4],r[0].reaction[5]);
    return 0;
}
