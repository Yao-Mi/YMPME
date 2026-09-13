#ifndef DEMO_TERRAIN_IO_H
#define DEMO_TERRAIN_IO_H
#include "MODEL/TERRAIN/YMPME_Terrain.h"
int Terrain_SavePGM(const pme_terrain* t,const char* path);
int Terrain_LoadPGM(pme_terrain* t,const char* path);
#endif
