#include "scene.h"
#include <stdio.h>
#include <string.h>
#define CHECK(x) do { if(!(x)) { fprintf(stderr,"line %d: %s\n",__LINE__,#x); return 1; } } while(0)
#define NEAR(a,b,e) CHECK(fabsf((a)-(b))<(e))
int main(void)
{
	pme_drive c,saved;
	car_scene scene;
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
	CHECK(CarScene_Init(&scene));
	{
		PME_Body chassis=NULL,wheel=NULL;
		pme_contact contact;
		for(i=0;i<scene.meshCount;++i)
		{
			if(scene.meshes[i].collider && !scene.meshes[i].wheel && !chassis) chassis=scene.meshes[i].collider;
			if(scene.meshes[i].collider && scene.meshes[i].wheel && !wheel) wheel=scene.meshes[i].collider;
		}
		CHECK(chassis && wheel);
		CHECK(!YMPME_World_Collide(&scene.world,chassis,wheel,&contact));
		CHECK(scene.world.collisionStats.filteredPairs==1 && scene.world.collisionStats.aabbTests==0);
		CHECK(scene.world.bodyCount>10);
	}
	/* 初始位置正前方检查点可达，继续直走应被北侧围栏阻挡。 */
	for(i=0;i<1600;++i) CHECK(CarScene_Step(&scene,2,2,0,dt));
	CHECK(scene.targets>=1 && scene.hits>=1);
	CHECK(scene.car.z<7.1f && scene.car.z>7.0f);
	NEAR(scene.car.x,0,0.0001f);
	CarScene_Reset(&scene);
	CHECK(scene.targets==0 && scene.hits==0 && scene.car.z==0);
	/* 在箱体下方前进，不能穿过障碍。 */
	scene.car.x=2; scene.car.z=-2;
	YMPME_Body_SetPosition(scene.proxy,YMPME_Vec3(2,0,-2));
	for(i=0;i<600;++i) CHECK(CarScene_Step(&scene,2,2,0,dt));
	CHECK(scene.car.z < -0.32f);
	CHECK(scene.world.collisionStats.meshTests>0);
	CarScene_SetMeshPrecision(&scene,0);
	CHECK(scene.car.z==0 && scene.world.config.meshPrecision==0);
	for(i=0;i<200;++i) CHECK(CarScene_Step(&scene,1,1,0,dt));
	CHECK(scene.world.collisionStats.meshTests==0);
	CarScene_SetMeshPrecision(&scene,1);
	CarScene_Destroy(&scene);
	puts("PASS differential drive: straight, pivot, arc, caster, collision, checkpoints");
	return 0;
}
