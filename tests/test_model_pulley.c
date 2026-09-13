#include "MODEL/PULLEY/YMPME_Pulley.h"
#include <stdio.h>
int main(void){pme_pulley p;YMPME_Pulley_Init(&p);
 for(int i=0;i<600;++i)YMPME_Pulley_Step(&p,0,1.0/600);
 if(fabs(p.tension-8*9.81/4)>1e-6)return 1;
 double y=p.loadY,q=p.freeY;
 for(int i=0;i<600;++i)YMPME_Pulley_Step(&p,1,1.0/600);
 if(p.loadY<y+.1||fabs(4*(p.loadY-y)+p.freeY-q)>1e-9)return 1;
 y=p.loadY;p.hold=0;for(int i=0;i<120;++i)YMPME_Pulley_Step(&p,0,1.0/600);
 if(p.loadY>=y)return 1;puts("PASS standalone 4:1 pulley: load balance, lift, release");return 0;}
