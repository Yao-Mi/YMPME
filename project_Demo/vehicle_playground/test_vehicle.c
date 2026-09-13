#include "terrain.h"
#include <stdio.h>
#include <math.h>
int main(void) {
 pme_terrain t,copy;YMPME_Terrain_InitHills(&t);
 if(!Terrain_SavePGM(&t,"terrain_test.pgm")||!Terrain_LoadPGM(&copy,"terrain_test.pgm"))return 1;
 remove("terrain_test.pgm");
 for(int i=0;i<PME_TERRAIN_N*PME_TERRAIN_N;++i)if(fabsf(copy.height[i]-t.height[i])>=.003f)return 1;
 puts("PASS demo terrain PGM round trip");return 0;
}
