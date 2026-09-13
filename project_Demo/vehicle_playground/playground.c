#include "playground.h"
#include "YMGRE_BasicMesh_Gener.h"
#include "YMGRE_Rendering_Pipeline.h"
#include "YMGRE_Camera.h"
#include "YMGRE_Creat.h"
#include "YMGRE_Free.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define U 30.0f
#define PI 3.14159265359f
static pme_rotation multiply(pme_rotation a,pme_rotation b)
{
	pme_rotation r; int i;
	for(i=0;i<3;++i) r.axis[i]=YMPME_Rotation_Apply(a,b.axis[i]);
	return r;
}
static pme_rotation yawRotation(float a)
{
	pme_rotation r={{{cosf(a),0,-sinf(a)},{0,1,0},{sinf(a),0,cosf(a)}}}; return r;
}
static pme_rotation xRotation(float a)
{
	pme_rotation r={{{1,0,0},{0,cosf(a),sinf(a)},{0,-sinf(a),cosf(a)}}}; return r;
}
static pme_rotation axleRotation(void)
{
	pme_rotation r={{{0,-1,0},{1,0,0},{0,0,1}}}; return r;
}
static void transform(fleet_part* m,pme_vec3 position,pme_rotation rotation,pme_vec3 scale)
{
	GRE_Object4d o=m->object; int i;
	o->WorldCoordinate=(gre_fvector4d){position.x*U,position.y*U,position.z*U,1};
	for(i=0;i<o->pointNum;++i)
	{
		gre_fvector4d a=m->local[i].pos;
		pme_vec3 b=YMPME_Rotation_Apply(rotation,YMPME_Vec3(a.x*scale.x,a.y*scale.y,a.z*scale.z));
		o->pointList[i]=m->local[i];
		o->pointList[i].pos=(gre_fvector4d){b.x+position.x*U,b.y+position.y*U,b.z+position.z*U,1};
	}
	for(i=0;i<o->polygonNum;++i)
	{
		gre_fvector4d a=m->normals[i];
		pme_vec3 n=YMPME_Rotation_Apply(rotation,YMPME_Vec3(a.x/scale.x,a.y/scale.y,a.z/scale.z));
		o->polygonList[i].pN=(gre_fvector4d){n.x,n.y,n.z,0};
	}
}
static int bindMesh(playground* p,fleet_part* m)
{
	GRE_Object4d o=m->object; int i,j;
	pme_body_desc d=YMPME_Body_DefaultDesc();
	m->vertices=malloc((size_t)o->pointNum*sizeof(*m->vertices));
	m->indices=malloc((size_t)o->polygonNum*3*sizeof(*m->indices));
	m->faces=malloc((size_t)o->polygonNum*sizeof(*m->faces));
	m->edges=malloc((size_t)o->polygonNum*3*sizeof(*m->edges));
	if(!m->vertices||!m->indices||!m->faces||!m->edges) return 0;
	for(i=0;i<o->pointNum;++i) m->vertices[i]=YMPME_Vec3(m->local[i].pos.x/U,m->local[i].pos.y/U,m->local[i].pos.z/U);
	for(i=0;i<o->polygonNum;++i)
	{
		if(o->polygonList[i].num!=3) return 0;
		for(j=0;j<3;++j) m->indices[3*i+j]=o->polygonList[i].index[j];
	}
	if(YMPME_ConvexMesh_Init(&m->mesh,m->vertices,(size_t)o->pointNum,m->indices,(size_t)o->polygonNum,
		m->faces,(size_t)o->polygonNum,m->edges,(size_t)o->polygonNum*3)!=PME_OK) return 0;
	d.shape=PME_CONVEX_MESH; d.mesh=&m->mesh;
	d.type=m->owner<0?PME_STATIC:PME_KINEMATIC;
	d.ignoreAssembly=m->owner<0?0:(uint32_t)m->owner+1;
	d.collisionGroup=m->owner<0?0:(m->kind==PART_WHEEL?2:1);
	d.position=m->offset;
	m->collider=YMPME_World_AddBody(&p->world,&d);
	return m->collider!=NULL;
}
static fleet_part* part(playground* p,GRE_Object4d obj,int owner,part_kind kind,pme_vec3 offset,int support,int index,int collide)
{
	fleet_part* m; int i;
	if(!obj) return NULL;
	if(p->partCount>=FLEET_PART_MAX) { YMGRE_Free_Object(obj); return NULL; }
	m=&p->part[p->partCount++]; m->object=obj; m->owner=owner; m->kind=kind; m->offset=offset; m->support=support; m->index=index;
	m->local=malloc((size_t)obj->pointNum*sizeof(*m->local));
	m->normals=malloc((size_t)obj->polygonNum*sizeof(*m->normals));
	if(!m->local||!m->normals) return NULL;
	memcpy(m->local,obj->pointList,(size_t)obj->pointNum*sizeof(*m->local));
	for(i=0;i<obj->polygonNum;++i) m->normals[i]=obj->polygonList[i].pN;
	obj->boundType=GRE_Bounding_Sphere_R;
	transform(m,offset,YMPME_Rotation_Identity(),YMPME_Vec3(1,1,1));
	YMGRE_List_Append(&p->objects,sizeof(*obj),obj);
	if(collide&&!bindMesh(p,m)) return NULL;
	return m;
}
static fleet_part* box(playground* p,int owner,pme_vec3 at,pme_vec3 size,GRErgb24 color,int collide)
{
	return part(p,YMGRE_MeshGener_Box(size.x*U,size.y*U,size.z*U,color,"body",""),owner,
		owner<0?PART_FIXED:PART_BODY,at,-1,0,collide);
}
static void normalFromVertices(GRE_Object4d o,int i)
{
	GRE_Polygon4d face=&o->polygonList[i];
	gre_fvector4d a=o->pointList[face->index[0]].pos,b=o->pointList[face->index[1]].pos,c=o->pointList[face->index[2]].pos;
	pme_vec3 n=YMPME_VecCross(YMPME_Vec3(b.x-a.x,b.y-a.y,b.z-a.z),YMPME_Vec3(c.x-a.x,c.y-a.y,c.z-a.z));
	face->pN=(gre_fvector4d){n.x,n.y,n.z,0};
}
static int terrainMeshes(playground* p)
{
	int tx,tz,i;
	for(tz=0;tz<8;++tz) for(tx=0;tx<8;++tx)
	{
		float cx=PME_TERRAIN_MIN+(tx*8+4)*PME_TERRAIN_STEP,cz=PME_TERRAIN_MIN+(tz*8+4)*PME_TERRAIN_STEP;
		GRE_Object4d o=YMGRE_MeshGener_RectPlane(3*U,3*U,8,8,(GRErgb24){90,117,89},"heightfield","" );
		if(!o) return 0;
		for(i=0;i<o->pointNum;++i)
		{
			int ix=tx*8+i%9,iz=tz*8+i/9;
			o->pointList[i].pos.y=p->terrain.height[iz*PME_TERRAIN_N+ix]*U;
		}
		for(i=0;i<o->polygonNum;++i)
		{
			int cell=i/2,gx=tx*8+cell%8,gz=tz*8+cell/8;
			float h=p->terrain.height[gz*PME_TERRAIN_N+gx];
			int bright=((gx/2+gz/2)%2)*8;
			o->polygonList[i].planeColor=(GRErgb24){(uint8)(80+bright+h*50),(uint8)(110+bright+h*20),(uint8)(91+bright-h*15)};
			normalFromVertices(o,i);
		}
		o->BoundingSphereR=3.0f*U;
		if(!part(p,o,-1,PART_FIXED,YMPME_Vec3(cx,0,cz),-1,0,0)) return 0;
	}
	return 1;
}
static int vehicleMeshes(playground* p,int owner)
{
	pme_vehicle* v=&p->vehicle[owner];
	GRErgb24 color=owner==0?(GRErgb24){220,167,65}:owner==1?(GRErgb24){54,170,221}:(GRErgb24){218,89,69};
	pme_vec3 bodySize=owner==0?YMPME_Vec3(1.0f,0.24f,1.85f):owner==1?YMPME_Vec3(.82f,.18f,1.05f):YMPME_Vec3(1.0f,.24f,1.8f);
	int i,j,side;
	if(!box(p,owner,YMPME_Vec3(0,0,0),bodySize,color,1) ||
		!box(p,owner,YMPME_Vec3(0,.20f,-.12f),YMPME_Vec3(bodySize.x*.60f,.20f,bodySize.z*.5f),(GRErgb24){38,53,63},0) ||
		!box(p,owner,YMPME_Vec3(0,.09f,bodySize.z*.47f),YMPME_Vec3(bodySize.x*.65f,.10f,.08f),(GRErgb24){249,233,165},0)) return 0;
	for(i=0;i<v->supportCount;++i)
	{
		float radius=v->support[i].radius,width=owner==0?.26f:(owner==1&&i==2?.12f:.18f);
		fleet_part* tire=part(p,YMGRE_MeshGener_Cylinder(radius*U,width*U,16,(GRErgb24){35,43,48},"wheel",""),
			owner,PART_WHEEL,YMPME_Vec3(0,0,0),i,0,owner!=0);
		fleet_part* hub=part(p,YMGRE_MeshGener_Cylinder(radius*.55f*U,(width+.015f)*U,12,color,"hub",""),
			owner,PART_WHEEL,YMPME_Vec3(0,0,0),i,0,0);
		if(!tire||!hub) return 0;
		for(j=0;j<hub->object->polygonNum;++j) if(j%8<4) hub->object->polygonList[j].planeColor=(GRErgb24){198,213,220};
		if(owner!=0)
		{
			if(owner==2 && !box(p,owner,YMPME_Vec3(v->support[i].x*.87f,0,v->support[i].z),
				YMPME_Vec3(.20f,.045f,.07f),(GRErgb24){126,137,145},0)) return 0;
			if(!part(p,YMGRE_MeshGener_Cylinder(.025f*U,1.0f*U,8,(GRErgb24){206,215,225},"damper",""),
				owner,PART_ROD,YMPME_Vec3(0,0,0),i,0,0)) return 0;
			if(owner==2) for(j=0;j<5;++j)
				if(!part(p,YMGRE_MeshGener_Torus(.065f*U,.012f*U,12,5,(GRErgb24){247,211,73},"spring-coil",""),
					owner,PART_COIL,YMPME_Vec3(0,0,0),i,j,0)) return 0;
		}
	}
	if(owner==1)
	{
		if(!part(p,YMGRE_MeshGener_Box(.20f*U,.05f*U,.20f*U,(GRErgb24){194,206,214},"caster-fork",""),
			owner,PART_FORK,YMPME_Vec3(0,.12f,0),2,0,0)) return 0;
	}
	if(owner==0)
	{
		for(side=0;side<2;++side)
		{
			fleet_part* envelope=box(p,owner,YMPME_Vec3(side?.65f:-.65f,-.12f,0),YMPME_Vec3(.34f,.53f,1.96f),color,1);
			if(!envelope) return 0;
			envelope->object->isVisible=0;
			for(j=0;j<28;++j)
				if(!part(p,YMGRE_MeshGener_Box(.32f*U,.055f*U,.14f*U,j%2?(GRErgb24){65,70,62}:(GRErgb24){89,94,77},"track-link",""),
					owner,PART_TREAD,YMPME_Vec3(side?.65f:-.65f,0,0),side,j,0)) return 0;
		}
	}
	return 1;
}
static void partPose(playground* p,fleet_part* m,pme_vec3* pos,pme_rotation* rotation,pme_vec3* scale)
{
	pme_vehicle* v=&p->vehicle[m->owner];
	pme_rotation body=YMPME_Vehicle_Rotation(v);
	pme_vec3 origin=YMPME_Vec3(v->drive.x,v->y,v->drive.z);
	*scale=YMPME_Vec3(1,1,1); *rotation=body;
	*pos=YMPME_VecAdd(origin,YMPME_Rotation_Apply(body,m->offset));
	if(m->kind==PART_WHEEL)
	{
		*pos=v->support[m->support].position;
		*rotation=multiply(multiply(multiply(body,yawRotation(v->wheelSteer[m->support])),xRotation(v->wheelAngle[m->support])),axleRotation());
	}
	else if(m->kind==PART_ROD||m->kind==PART_COIL)
	{
		pme_wheel_support* s=&v->support[m->support];
		float ratio=m->kind==PART_ROD?.5f:(m->index+1)/6.0f;
		*pos=s->position; pos->y+=s->length*ratio;
		*rotation=YMPME_Rotation_Identity();
		if(m->kind==PART_ROD) scale->y=fmaxf(.02f,s->length);
	}
	else if(m->kind==PART_FORK)
	{
		*pos=YMPME_VecAdd(v->support[2].position,m->offset);
		*rotation=multiply(body,yawRotation(v->drive.casterAngle));
	}
	else if(m->kind==PART_TREAD)
	{
		float radius=.25f,total=PME_TRACK_LOOP_LENGTH;
		float path=fmodf(m->index*(total/28)-v->trackDistance[m->support],total),y,z,dy,dz,h=0;
		if(path<0) path+=total;
		if(path<1.4f) { y=-.22f-radius; z=-.7f+path; dy=0; dz=1; }
		else if((path-=1.4f)<PI*radius)
		{ float a=-PI/2+path/radius; y=-.22f+radius*sinf(a); z=.7f+radius*cosf(a); dy=cosf(a); dz=-sinf(a); }
		else if((path-=PI*radius)<1.4f) { y=-.22f+radius; z=.7f-path; dy=0; dz=-1; }
		else
		{ float a=PI/2+(path-1.4f)/radius; y=-.22f+radius*sinf(a); z=-.7f+radius*cosf(a); dy=cosf(a); dz=-sinf(a); }
		*pos=YMPME_VecAdd(origin,YMPME_Rotation_Apply(body,YMPME_Vec3(m->offset.x,y,z)));
		if(y<-.22f && YMPME_Terrain_Sample(&p->terrain,pos->x,pos->z,&h,NULL)) pos->y=fmaxf(pos->y,h+.035f);
		*rotation=multiply(body,xRotation(atan2f(-dy,dz)));
	}
}
static void syncColliders(playground* p)
{
	int i;
	for(i=0;i<p->partCount;++i)
	{
		fleet_part* m=&p->part[i];
		pme_vec3 pos,scale; pme_rotation r;
		if(m->owner<0||!m->collider) continue;
		partPose(p,m,&pos,&r,&scale);
		YMPME_Body_SetPosition(m->collider,pos); YMPME_Body_SetRotation(m->collider,r);
	}
}
static int overlaps(playground* p,int owner)
{
	int i; size_t j;
	for(i=0;i<p->partCount;++i)
	{
		fleet_part* m=&p->part[i];
		if(m->owner!=owner||!m->collider) continue;
		for(j=0;j<p->world.capacity;++j)
		{
			pme_contact c;
			if(!p->world.bodies[j].active) continue;
			if(YMPME_World_Collide(&p->world,m->collider,&p->world.bodies[j],&c)&&c.penetration>.002f) return 1;
		}
	}
	return 0;
}
int Playground_Init(playground* p,const char* heightmap)
{
	int i; pme_world_desc d=YMPME_World_DefaultDesc();
	memset(p,0,sizeof(*p)); YMPME_Terrain_InitHills(&p->terrain);
	if(heightmap&&!Terrain_LoadPGM(&p->terrain,heightmap)) return 0;
	d.gravity=YMPME_Vec3(0,0,0);
	if(YMPME_World_Init(&p->world,p->bodies,64,&d)!=PME_OK) return 0;
	for(i=0;i<3;++i) { YMPME_Vehicle_Init(&p->vehicle[i],(pme_vehicle_kind)i,(i-1)*4,-6.4f,&p->terrain); p->vehicle[i].worldLimit=10.4f; }
	p->selected=2;
	if(!terrainMeshes(p)) return 0;
	for(i=0;i<3;++i) if(!vehicleMeshes(p,i)) return 0;
	for(i=-1;i<=1;i+=2)
	{
		int segment;
		fleet_part* xWall=box(p,-1,YMPME_Vec3(i*11.5f,.30f,0),YMPME_Vec3(.2f,.6f,23.2f),(GRErgb24){120,150,175},1);
		fleet_part* zWall=box(p,-1,YMPME_Vec3(0,.30f,i*11.5f),YMPME_Vec3(23.2f,.6f,.2f),(GRErgb24){120,150,175},1);
		if(!xWall||!zWall) return 0;
		xWall->object->isVisible=zWall->object->isVisible=0;
		/* 长围栏分段显示，避免低机位时超长三角形跨越相机近面。 */
		for(segment=0;segment<12;++segment)
		{
			float center=-11.6f+(segment+.5f)*(23.2f/12);
			if(!box(p,-1,YMPME_Vec3(i*11.5f,.30f,center),YMPME_Vec3(.2f,.6f,23.2f/12),(GRErgb24){120,150,175},0)||
				!box(p,-1,YMPME_Vec3(center,.30f,i*11.5f),YMPME_Vec3(23.2f/12,.6f,.2f),(GRErgb24){120,150,175},0)) return 0;
		}
	}
	/* 停车区侧边障碍用于验证车辆间/环境 Mesh 过滤，不阻挡三条悬挂测试路线。 */
	if(!box(p,-1,YMPME_Vec3(-8,.45f,-3),YMPME_Vec3(1,.9f,1.5f),(GRErgb24){224,119,64},1)||
		!box(p,-1,YMPME_Vec3(8,.45f,-3),YMPME_Vec3(1,.9f,1.5f),(GRErgb24){224,119,64},1)) return 0;
	p->ambient=YMGRE_Creat_Light(0,GRE_GlobalLight,(GRErgb24){255,255,255},.42f);
	p->keyLight=YMGRE_Creat_Light(1,GRE_PointLight,(GRErgb24){255,244,216},.85f);
	if(!p->ambient||!p->keyLight) return 0;
	p->keyLight->pos=(gre_fvector4d){-200,500,-100,1}; p->keyLight->proper.kc0=1;
	p->keyLight->proper.kc1=p->keyLight->proper.kc2=0;
	YMGRE_List_Append(&p->lights,sizeof(*p->ambient),p->ambient); YMGRE_List_Append(&p->lights,sizeof(*p->keyLight),p->keyLight);
	p->camera=YMGRE_Creat_Camera(0,900,540,38,38,26,26); p->workspace=YMGRE_Creat_RenderWorkspace();
	if(!p->camera||!p->workspace) return 0;
	YMGRE_Camera_Frustum_Init(p->camera,1,2000); p->camera->wireFrame=GRE_Render_Solid;
	syncColliders(p); return 1;
}
void Playground_Reset(playground* p)
{
	int i;
	for(i=0;i<3;++i) { YMPME_Vehicle_Init(&p->vehicle[i],(pme_vehicle_kind)i,(i-1)*4,-6.4f,&p->terrain); p->vehicle[i].worldLimit=10.4f; }
	p->elapsed=0; p->contacts=0; p->world.stepCount=0; syncColliders(p);
}
int Playground_Step(playground* p,float throttle,float steer,int brake,float dt)
{
	int i,k; pme_vehicle old[3];
	memcpy(old,p->vehicle,sizeof(old));
	if(YMPME_World_Step(&p->world,dt)!=PME_OK) return 0;
	for(i=0;i<3;++i)
		if(!YMPME_Vehicle_Step(&p->vehicle[i],&p->terrain,i==p->selected?throttle:0,i==p->selected?steer:0,i==p->selected?brake:1,dt)) return 0;
	syncColliders(p);
	for(i=0;i<3;++i)
	{
		pme_vehicle* v=&p->vehicle[i];
		if(overlaps(p,i))
		{
			float low=0,high=1,x=v->drive.x,z=v->drive.z,yaw=v->drive.yaw;
			++p->contacts;
			for(k=0;k<6;++k)
			{
				float t=(low+high)*.5f;
				v->drive.x=old[i].drive.x+(x-old[i].drive.x)*t;
				v->drive.z=old[i].drive.z+(z-old[i].drive.z)*t;
				v->drive.yaw=old[i].drive.yaw+remainderf(yaw-old[i].drive.yaw,2*PI)*t;
				YMPME_Vehicle_UpdateSupports(v,&p->terrain); syncColliders(p);
				if(overlaps(p,i)) high=t; else low=t;
			}
			v->drive.x=old[i].drive.x+(x-old[i].drive.x)*low; v->drive.z=old[i].drive.z+(z-old[i].drive.z)*low;
			v->drive.yaw=old[i].drive.yaw+remainderf(yaw-old[i].drive.yaw,2*PI)*low;
			YMPME_Vehicle_UpdateSupports(v,&p->terrain); syncColliders(p);
		}
	}
	p->elapsed+=dt; return 1;
}
void Playground_Render(playground* p,int cameraMode,int wireframe)
{
	int i; gre_fvector4d eye,target; pme_vehicle* selected=&p->vehicle[p->selected];
	for(i=0;i<p->partCount;++i)
	{
		fleet_part* m=&p->part[i]; pme_vec3 pos,scale; pme_rotation r;
		if(m->owner<0) continue;
		partPose(p,m,&pos,&r,&scale); transform(m,pos,r,scale);
		m->object->wireFrame=(uint8)(wireframe&&m->owner==p->selected&&m->kind==PART_BODY);
	}
	if(cameraMode==1) { eye=(gre_fvector4d){380,540,-490,1}; target=(gre_fvector4d){0,0,0,1}; }
	else if(cameraMode==2)
	{
		eye=(gre_fvector4d){selected->drive.x*U+145,selected->y*U+35,selected->drive.z*U-25,1};
		target=(gre_fvector4d){selected->drive.x*U,selected->y*U-8,selected->drive.z*U,1};
	}
	else
	{
		eye=(gre_fvector4d){selected->drive.x*U+110,selected->y*U+110,selected->drive.z*U-165,1};
		target=(gre_fvector4d){selected->drive.x*U,selected->y*U,selected->drive.z*U+25,1};
	}
	YMGRE_UVNCamera_PositionInit(p->camera,&eye,&target,NULL,0);
	YMGRE_Camera_TanglePipline_RenderingWithWorkspace(p->camera,&p->lights,&p->objects,&p->materials,p->workspace);
}
static void keep(void* data) { (void)data; }
void Playground_Destroy(playground* p)
{
	int i;
	if(p->workspace) YMGRE_Free_RenderWorkspace(p->workspace);
	if(p->camera) YMGRE_Free_Camera(p->camera);
	YMGRE_List_Clear(&p->objects,keep); YMGRE_List_Clear(&p->lights,keep);
	for(i=0;i<p->partCount;++i)
	{
		fleet_part* m=&p->part[i]; free(m->local); free(m->normals); free(m->vertices); free(m->indices); free(m->faces); free(m->edges);
		if(m->object) YMGRE_Free_Object(m->object);
	}
	if(p->ambient) YMGRE_Free_Light(p->ambient);
	if(p->keyLight) YMGRE_Free_Light(p->keyLight);
	if(p->world.bodies) YMPME_World_Destroy(&p->world);
}
