#include "MODEL/DRIVE/YMPME_Drive.h"
#include <stdio.h>
#include <string.h>
#define CHECK(x) do { if(!(x)) { fprintf(stderr,"line %d: %s\n",__LINE__,#x); return 1; } } while(0)
#define NEAR(a,b,e) CHECK(fabsf((a)-(b))<(e))
int main(void)
{
	pme_drive c,saved;
	int i;
	const float dt=1.0f/120;
	YMPME_Drive_Reset(&c); c.left=c.right=1;
	for(i=0;i<120;++i) CHECK(YMPME_Drive_Step(&c,1,1,dt)==PME_OK);
	NEAR(c.z,1,0.00001f); NEAR(c.x,0,0.00001f); NEAR(c.yaw,0,0.00001f);
	YMPME_Drive_Reset(&c); c.left=1; c.right=-1;
	for(i=0;i<120;++i) CHECK(YMPME_Drive_Step(&c,1,-1,dt)==PME_OK);
	NEAR(c.x,0,0.00001f); NEAR(c.z,0,0.00001f); NEAR(c.yaw,2/c.track,0.00002f);
	YMPME_Drive_Reset(&c); c.left=1; c.right=0;
	for(i=0;i<120;++i) CHECK(YMPME_Drive_Step(&c,1,0,dt)==PME_OK);
	NEAR(c.x,0.5f*c.track*(1-cosf(1/c.track)),0.00002f);
	NEAR(c.z,0.5f*c.track*sinf(1/c.track),0.00002f);
	YMPME_Drive_Reset(&c); c.casterAngle=1; c.left=c.right=1;
	for(i=0;i<360;++i) CHECK(YMPME_Drive_Step(&c,1,1,dt)==PME_OK);
	NEAR(c.casterAngle,0,0.001f);
	c.left=c.right=-1;
	for(i=0;i<600;++i) CHECK(YMPME_Drive_Step(&c,-1,-1,dt)==PME_OK);
	NEAR(fabsf(c.casterAngle),3.14159265f,0.001f);
	saved=c;
	CHECK(YMPME_Drive_Step(&c,NAN,0,dt)==PME_INVALID_ARGUMENT);
	CHECK(memcmp(&c,&saved,sizeof(c))==0);
	puts("PASS reusable differential drive model"); return 0;
}
