#include "MODEL/VEHICLE/YMPME_Vehicle.h"
#include <stdio.h>
#include <string.h>
#define CHECK(x) do { if(!(x)) { fprintf(stderr,"line %d: %s\n",__LINE__,#x); return 1; } } while(0)
#define NEAR(a,b,e) CHECK(fabsf((a)-(b))<(e))
int main(void)
{
	pme_terrain terrain,flat={0},copy;
	pme_vehicle v; int kind,i,j; float h; pme_vec3 n;
	YMPME_Terrain_InitHills(&terrain);
	for(i=0;i<PME_TERRAIN_N;++i) for(j=0;j<PME_TERRAIN_N;++j)
	{
		CHECK(YMPME_Terrain_Sample(&terrain,PME_TERRAIN_MIN+j*PME_TERRAIN_STEP,PME_TERRAIN_MIN+i*PME_TERRAIN_STEP,&h,&n));
		NEAR(h,terrain.height[i*PME_TERRAIN_N+j],1e-5f);
		CHECK(n.y>0);
	}
	CHECK(!YMPME_Terrain_Sample(&terrain,100,0,&h,NULL));
	/* 一个非共面的单元，确认采用三角插值而非双线性。 */
	copy=flat; copy.height[1]=1; copy.height[PME_TERRAIN_N]=2; copy.height[PME_TERRAIN_N+1]=4;
	CHECK(YMPME_Terrain_Sample(&copy,-12+.25f*PME_TERRAIN_STEP,-12+.25f*PME_TERRAIN_STEP,&h,NULL)); NEAR(h,.75f,1e-4f);
	CHECK(YMPME_Terrain_Sample(&copy,-12+.75f*PME_TERRAIN_STEP,-12+.75f*PME_TERRAIN_STEP,&h,NULL)); NEAR(h,2.75f,1e-4f);

	for(kind=0;kind<3;++kind)
	{
		float force=0;
		YMPME_Vehicle_Init(&v,(pme_vehicle_kind)kind,(kind-1)*4,-6.4f,&flat);
		for(i=0;i<1200;++i) CHECK(YMPME_Vehicle_Step(&v,&flat,0,0,1,1.0f/120));
		NEAR(v.vy,0,.001f); CHECK(fabsf(v.pitch)<.03f&&fabsf(v.roll)<.001f);
		CHECK(v.contactCount==v.supportCount);
		for(i=0;i<v.supportCount;++i) force+=v.support[i].force;
		NEAR(force,v.mass*9.81f,1);
	}
	/* 四轮前转向：静止时不原地转圈，前轮内外转角不同。 */
	YMPME_Vehicle_Init(&v,PME_VEH_FOUR_WHEEL,4,-6.4f,&terrain);
	for(i=0;i<120;++i) CHECK(YMPME_Vehicle_Step(&v,&terrain,0,1,0,1.0f/120));
	NEAR(v.drive.yaw,0,1e-6f); CHECK(v.wheelSteer[1]>v.wheelSteer[0]);
	/* 驶过交叉轴山包，必须存在独立压缩和车身姿态响应。 */
	YMPME_Vehicle_Init(&v,PME_VEH_FOUR_WHEEL,4,-6.4f,&terrain);
	{
		float difference=0,tilt=0,maxHeight=0;
		for(i=0;i<850;++i)
		{
			float lo=1,hi=0;
			CHECK(YMPME_Vehicle_Step(&v,&terrain,.7f,0,0,1.0f/120));
			for(j=0;j<4;++j)
			{
				lo=fminf(lo,v.support[j].compression); hi=fmaxf(hi,v.support[j].compression);
				CHECK(v.support[j].length<=v.support[j].rest+1e-6f);
				CHECK(v.support[j].compression>=0 && v.support[j].compression<=v.support[j].travel);
				CHECK(v.support[j].position.y>=v.support[j].groundHeight+v.support[j].radius-.08f);
			}
			difference=fmaxf(difference,hi-lo); tilt=fmaxf(tilt,fabsf(v.pitch)+fabsf(v.roll)); maxHeight=fmaxf(maxHeight,v.y);
		}
		CHECK(difference>.05f && tilt>.1f && maxHeight>1.0f);
	}
	/* 抬离地面后没有悬挂拉力，车身靠重力下落而非瞬间贴地。 */
	YMPME_Vehicle_Init(&v,PME_VEH_FOUR_WHEEL,0,-6,&flat); v.y+=2;
	CHECK(YMPME_Vehicle_Step(&v,&flat,0,0,1,1.0f/120)); CHECK(v.contactCount==0&&v.vy<0&&v.y>2);
	for(i=0;i<1200;++i) CHECK(YMPME_Vehicle_Step(&v,&flat,0,0,1,1.0f/120));
	NEAR(v.vy,0,.001f); CHECK(v.contactCount==4); NEAR(v.y,.698687f,.001f);
	/* 所有车完整通过各自路线；履带位移在轮子转角回绕后仍连续。 */
	for(kind=0;kind<3;++kind)
	{
		YMPME_Vehicle_Init(&v,(pme_vehicle_kind)kind,(kind-1)*4,-6.4f,&terrain);
		for(i=0;i<1100;++i)
		{
			float previous=v.trackDistance[0];
			CHECK(YMPME_Vehicle_Step(&v,&terrain,.7f,0,0,1.0f/120));
			CHECK(v.y>0&&v.y<2&&fabsf(v.pitch)<=.60f&&fabsf(v.roll)<=.60f);
			if(kind==0) NEAR(remainderf(v.trackDistance[0]-previous,PME_TRACK_LOOP_LENGTH),v.drive.left/120,1e-5f);
		}
		CHECK(v.drive.z>5);
	}
	puts("PASS terrain triangulation, PGM, static spring balance, 3 hill routes, steering, suspension, landing and continuous tracks");
	return 0;
}
