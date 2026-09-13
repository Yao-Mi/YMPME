#include "MODEL/TERRAIN/YMPME_Terrain.h"
#include <stdio.h>
int main(void){pme_terrain t={0};float h=77;pme_vec3 n;
 t.height[1]=1;t.height[PME_TERRAIN_N]=2;t.height[PME_TERRAIN_N+1]=4;
 if(!YMPME_Terrain_Sample(&t,-12+.25f*PME_TERRAIN_STEP,-12+.25f*PME_TERRAIN_STEP,&h,&n)||fabsf(h-.75f)>.0001f||n.y<=0)return 1;
 h=77;if(YMPME_Terrain_Sample(&t,100,0,&h,&n)||h!=77)return 1;
 YMPME_Terrain_InitHills(&t);if(!YMPME_Terrain_Sample(&t,-4,0,&h,&n)||h<.5f)return 1;
 puts("PASS standalone heightfield sampling and hills preset");return 0;}
