#include "terrain.h"
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
int Terrain_SavePGM(const pme_terrain* t,const char* path)
{
	FILE* f=fopen(path,"wb"); int i,ok=1;
	if(!f) return 0;
	fprintf(f,"P5\n# YMPME height field: black=0m white=1.2m\n65 65\n255\n");
	for(i=0;i<PME_TERRAIN_N*PME_TERRAIN_N;++i)
		if(fputc((int)lroundf(YMPME_Clamp(t->height[i]/1.2f,0,1)*255),f)==EOF) { ok=0; break; }
	if(fclose(f)!=0) ok=0;
	return ok;
}
static int token(FILE* f,char* out,size_t capacity)
{
	int c; size_t n=0;
	do
	{
		c=fgetc(f);
		if(c=='#') { while(c!=EOF&&c!='\n') c=fgetc(f); }
	} while(c!=EOF&&isspace((unsigned char)c));
	while(c!=EOF&&!isspace((unsigned char)c))
	{
		if(n+1>=capacity) return 0;
		out[n++]=(char)c; c=fgetc(f);
	}
	if(c=='\r') { c=fgetc(f); if(c!='\n'&&c!=EOF) ungetc(c,f); }
	out[n]=0; return n>0;
}
int Terrain_LoadPGM(pme_terrain* t,const char* path)
{
	FILE* f=fopen(path,"rb"); char text[32]; pme_terrain temp; int i,ok=0;
	if(!f) return 0;
	if(!token(f,text,sizeof(text))||strcmp(text,"P5")||!token(f,text,sizeof(text))||strcmp(text,"65")||
		!token(f,text,sizeof(text))||strcmp(text,"65")||!token(f,text,sizeof(text))||strcmp(text,"255")) goto done;
	for(i=0;i<PME_TERRAIN_N*PME_TERRAIN_N;++i) { int v=fgetc(f); if(v==EOF) goto done; temp.height[i]=v*(1.2f/255); }
	*t=temp; ok=1;
done:
	fclose(f); return ok;
}
