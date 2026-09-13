#include "MODEL/BELT/YMPME_Belt.h"
#include <stdio.h>
int main(void){pme_belt driven,free,ideal;YMPME_Belt_Init(&driven,1);YMPME_Belt_Init(&free,1);YMPME_Belt_Init(&ideal,0);free.mu=0;
 for(int i=0;i<3600;++i){YMPME_Belt_Step(&driven,1.0/600);YMPME_Belt_Step(&free,1.0/600);YMPME_Belt_Step(&ideal,1.0/600);
  if(driven.maxStretch>.04||!isfinite(driven.omega[1])||fabs(ideal.omega[1]-ideal.omega[0]*.65/.95)>1e-12)return 1;}
 if(driven.omega[1]>-.5||fabs(free.omega[1])>1e-12||driven.contacts<20)return 1;
 driven.target=1;for(int i=0;i<3600;++i)YMPME_Belt_Step(&driven,1.0/600);
 if(driven.omega[1]<.5)return 1;
 puts("PASS standalone belts: ratio, contact transmission, no-friction isolation, reverse");return 0;}
