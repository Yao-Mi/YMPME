#include "MODEL/SUSPENSION/YMPME_Suspension.h"
#include <stdio.h>
#define CHECK(x) do{if(!(x)){fprintf(stderr,"line %d: %s\n",__LINE__,#x);return 1;}}while(0)
int main(void){pme_suspension_desc d={1000,100,1,.3f,8,5000};float f;
 CHECK(YMPME_Suspension_Force(&d,.8f,.2f,&f)==PME_OK&&fabsf(f-220)<.001f);
 CHECK(YMPME_Suspension_Force(&d,1.1f,2,&f)==PME_OK&&f==0);
 CHECK(YMPME_Suspension_Force(&d,.8f,-4,&f)==PME_OK&&f==0);
 CHECK(YMPME_Suspension_Force(&d,.5f,0,&f)==PME_OK&&fabsf(f-1900)<.01f);
 CHECK(YMPME_Suspension_Force(&d,0,100,&f)==PME_OK&&f==5000);
 f=123;CHECK(YMPME_Suspension_Force(&d,NAN,0,&f)==PME_INVALID_ARGUMENT&&f==123);
 puts("PASS suspension force, rebound, bump stop and force cap");return 0;}
