#include "scene.h"
#include "YMGRE_BasicMesh_Gener.h"
#include "YMGRE_Rendering_Pipeline.h"
#include "YMGRE_Camera.h"
#include "YMGRE_Creat.h"
#include "YMGRE_Free.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#define U 30.0f
static const float checkpoints[CAR_TARGET_COUNT][2] = {{0, 2.8f}, {3.8f, 4.5f}, {4.5f, -2.5f}, {-3.5f, -3.5f}, {-4, 3.5f}};
static pme_vec3 rotateY(pme_vec3 p, float angle)
{
	float c = cosf(angle), s = sinf(angle);
	return YMPME_Vec3(c*p.x+s*p.z, p.y, -s*p.x+c*p.z);
}
static pme_vec3 rotateX(pme_vec3 p, float angle)
{
	float c = cosf(angle), s = sinf(angle);
	return YMPME_Vec3(p.x, c*p.y-s*p.z, s*p.y+c*p.z);
}
static pme_vec3 orient(pme_vec3 p, float yaw, float spin, int wheel)
{
	if (wheel)
	{
		/* GRE 圆柱沿 Y 轴生成；先转到 X 轮轴，再应用车轮滚转。 */
		p = YMPME_Vec3(p.y, -p.x, p.z);
		p = rotateX(p, spin);
	}
	return rotateY(p, yaw);
}

static void partPose(const car_scene* s,const car_mesh* m,pme_vec3* position,float* yaw,float* spin)
{
	pme_vec3 offset=m->offset;
	*yaw=s->car.yaw; *spin=0;
	if(m->caster)
	{
		offset=rotateY(offset,s->car.casterAngle);
		offset.z+=s->car.casterZ;
		*yaw+=s->car.casterAngle;
	}
	if(m->wheel==1) *spin=s->car.leftAngle;
	if(m->wheel==2) *spin=s->car.rightAngle;
	if(m->wheel==3) *spin=s->car.casterSpin;
	offset=rotateY(offset,s->car.yaw);
	*position=YMPME_VecAdd(YMPME_Vec3(s->car.x,0,s->car.z),offset);
}
static void syncColliders(car_scene* s)
{
	int i;
	for(i=0;i<s->meshCount;++i)
	{
		car_mesh* m=&s->meshes[i];
		pme_vec3 position;
		pme_rotation rotation;
		float yaw,spin;
		if(!m->collider) continue;
		partPose(s,m,&position,&yaw,&spin);
		rotation.axis[0]=orient(YMPME_Vec3(1,0,0),yaw,spin,m->wheel);
		rotation.axis[1]=orient(YMPME_Vec3(0,1,0),yaw,spin,m->wheel);
		rotation.axis[2]=orient(YMPME_Vec3(0,0,1),yaw,spin,m->wheel);
		YMPME_Body_SetPosition(m->collider,position);
		YMPME_Body_SetRotation(m->collider,rotation);
	}
}
static int bindCollisionMeshes(car_scene* s)
{
	int i,j,k;
	for(i=0;i<s->meshCount;++i)
	{
		car_mesh* m=&s->meshes[i];
		GRE_Object4d o=m->object;
		pme_body_desc d=YMPME_Body_DefaultDesc();
		if(!m->moving) continue;
		m->collisionVertices=malloc((size_t)o->pointNum*sizeof(*m->collisionVertices));
		m->collisionIndices=malloc((size_t)o->polygonNum*3*sizeof(*m->collisionIndices));
		m->faceAxes=malloc((size_t)o->polygonNum*sizeof(*m->faceAxes));
		m->edgeAxes=malloc((size_t)o->polygonNum*3*sizeof(*m->edgeAxes));
		if(!m->collisionVertices||!m->collisionIndices||!m->faceAxes||!m->edgeAxes) return 0;
		for(j=0;j<o->pointNum;++j) m->collisionVertices[j]=YMPME_Vec3(m->local[j].pos.x/U,m->local[j].pos.y/U,m->local[j].pos.z/U);
		for(j=0;j<o->polygonNum;++j)
		{
			if(o->polygonList[j].num!=3) return 0;
			for(k=0;k<3;++k) m->collisionIndices[j*3+k]=o->polygonList[j].index[k];
		}
		if(YMPME_ConvexMesh_Init(&m->collisionMesh,m->collisionVertices,(size_t)o->pointNum,
			m->collisionIndices,(size_t)o->polygonNum,m->faceAxes,(size_t)o->polygonNum,m->edgeAxes,(size_t)o->polygonNum*3)!=PME_OK)
		{
			fprintf(stderr,"Car collision mesh %d failed convex validation\n",i);
			return 0;
		}
		d.type=PME_KINEMATIC; /* 由同一差速装配体驱动，不把零件当成可散开的独立动态物体。 */
		d.shape=PME_CONVEX_MESH; d.mesh=&m->collisionMesh;
		d.collisionGroup=m->caster?3:(m->wheel?2:1);
		d.ignoreAssembly=1; /* 该车内部零件在 AABB 之前全部跳过。 */
		m->collider=YMPME_World_AddBody(&s->world,&d);
		if(!m->collider) return 0;
	}
	syncColliders(s);
	return 1;
}
static int overlapsWorld(car_scene* s)
{
	int i;
	size_t j;
	for(i=0;i<s->meshCount;++i)
	{
		PME_Body body=s->meshes[i].collider;
		if(!body) continue;
		for(j=0;j<s->world.capacity;++j)
		{
			pme_contact contact;
			const pme_body* other=&s->world.bodies[j];
			if(!other->active) continue;
			if(YMPME_World_Collide(&s->world,body,other,&contact) && contact.penetration>0.00001f) return 1;
		}
	}
	return 0;
}
static float blendAngle(float a,float b,float t)
{
	return a+remainderf(b-a,6.28318530718f)*t;
}
static pme_drive blendPose(const pme_drive* a,const pme_drive* b,float t)
{
	pme_drive c=*b;
	c.x=a->x+(b->x-a->x)*t; c.z=a->z+(b->z-a->z)*t;
	c.yaw=blendAngle(a->yaw,b->yaw,t);
	c.casterAngle=blendAngle(a->casterAngle,b->casterAngle,t);
	c.leftAngle=blendAngle(a->leftAngle,b->leftAngle,t);
	c.rightAngle=blendAngle(a->rightAngle,b->rightAngle,t);
	c.casterSpin=blendAngle(a->casterSpin,b->casterSpin,t);
	return c;
}
static void transform(car_mesh* m, pme_vec3 position, float yaw, float spin)
{
	int i;
	GRE_Object4d obj = m->object;
	obj->WorldCoordinate = (gre_fvector4d){position.x*U, position.y*U, position.z*U, 1};
	for (i = 0; i < obj->pointNum; ++i)
	{
		pme_vec3 p = orient(YMPME_Vec3(m->local[i].pos.x, m->local[i].pos.y, m->local[i].pos.z), yaw, spin, m->wheel);
		obj->pointList[i] = m->local[i];
		obj->pointList[i].pos = (gre_fvector4d){p.x+position.x*U, p.y+position.y*U, p.z+position.z*U, 1};
	}
	for (i = 0; i < obj->polygonNum; ++i)
	{
		pme_vec3 n = orient(YMPME_Vec3(m->normals[i].x, m->normals[i].y, m->normals[i].z), yaw, spin, m->wheel);
		obj->polygonList[i].pN = (gre_fvector4d){n.x, n.y, n.z, 0};
	}
}
static car_mesh* addMesh(car_scene* s, GRE_Object4d obj, pme_vec3 offset, int moving, int wheel, int caster)
{
	car_mesh* m;
	int i;
	if (!obj) return NULL;
	if (s->meshCount == CAR_MESH_MAX) { YMGRE_Free_Object(obj); return NULL; }
	m = &s->meshes[s->meshCount++];
	m->object = obj;
	m->local = malloc((size_t)obj->pointNum*sizeof(*m->local));
	m->normals = malloc((size_t)obj->polygonNum*sizeof(*m->normals));
	if (!m->local || !m->normals) return NULL;
	memcpy(m->local, obj->pointList, (size_t)obj->pointNum*sizeof(*m->local));
	for (i = 0; i < obj->polygonNum; ++i) m->normals[i] = obj->polygonList[i].pN;
	m->offset = offset;
	m->moving = moving;
	m->wheel = wheel;
	m->caster = caster;
	/* 旋转物体统一使用不随朝向改变的包围球做 GRE 剔除。 */
	obj->boundType = GRE_Bounding_Sphere_R;
	transform(m, offset, 0, 0);
	YMGRE_List_Append(&s->objects, sizeof(*obj), obj);
	return m;
}
static car_mesh* box(car_scene* s, float x, float y, float z, float w, float h, float d, GRErgb24 color, int moving)
{
	return addMesh(s, YMGRE_MeshGener_Box(w*U,h*U,d*U,color,"box",""), YMPME_Vec3(x,y,z),moving,0,0);
}
static int collider(car_scene* s, float x, float z, float w, float d, float y, float h)
{
	pme_body_desc b = YMPME_Body_DefaultDesc();
	b.type = PME_STATIC;
	b.shape = PME_BOX;
	b.position = YMPME_Vec3(x,y,z);
	b.halfExtents = YMPME_Vec3(w/2, h/2, d/2);
	b.friction = 0;
	return YMPME_World_AddBody(&s->world,&b) != NULL;
}
int CarScene_Init(car_scene* s)
{
	int i, side;
	pme_world_desc wd = YMPME_World_DefaultDesc();
	pme_body_desc bd = YMPME_Body_DefaultDesc();
	GRErgb24 floorColor = {49,64,79}, blue = {54,161,216};
	memset(s,0,sizeof(*s));
	YMPME_Drive_Reset(&s->car);
	wd.gravity = YMPME_Vec3(0,0,0);
	if (YMPME_World_Init(&s->world,s->bodies,64,&wd) != PME_OK) return 0;
	bd.type = PME_KINEMATIC;
	bd.mask = 0; /* 仅保存装配体根位置，不再使用整车球形碰撞。 */
	bd.ignoreAssembly = 1;
	bd.mass = 8;
	bd.friction = 0;
	s->proxy = YMPME_World_AddBody(&s->world,&bd);
	if (!s->proxy || !addMesh(s,YMGRE_MeshGener_RectPlane(16*U,16*U,16,16,floorColor,"floor",""),YMPME_Vec3(0,-0.03f,0),0,0,0)) return 0;
	/* 分块棋盘地面让位移易于观察，也避免跨越近裁剪面的细长网格条。 */
	for (i = 0; i < s->meshes[0].object->polygonNum; ++i)
	{
		int cell = i/2;
		if (((cell%16)+(cell/16))%2)
			s->meshes[0].object->polygonList[i].planeColor = (GRErgb24){61,78,93};
	}
	for (side=-1;side<=1;side+=2)
	{
		if (!box(s,side*7.5f,0.2f,0,0.2f,0.45f,15.2f,blue,0) ||
			!box(s,0,0.2f,side*7.5f,15.2f,0.45f,0.2f,blue,0) ||
			!collider(s,side*7.5f,0,0.2f,15.2f,0.2f,0.45f) || !collider(s,0,side*7.5f,15.2f,0.2f,0.2f,0.45f)) return 0;
	}
	{
		const float obstacles[][4] = {{-1.9f,2.8f,1.0f,2.0f},{2,1,0.8f,2.0f},{2,-3.6f,2.0f,0.8f},{-3.8f,0,1.2f,1.2f}};
		for(i=0;i<4;++i)
		{
			const float* o=obstacles[i];
			if(!box(s,o[0],0.4f,o[1],o[2],0.8f,o[3],(GRErgb24){215,115,66},0) || !collider(s,o[0],o[1],o[2],o[3],0.4f,0.8f)) return 0;
		}
	}
	for(i=0;i<CAR_TARGET_COUNT;++i)
	{
		car_mesh* marker=box(s,checkpoints[i][0],0.005f,checkpoints[i][1],0.8f,0.035f,0.8f,(GRErgb24){88,235,155},0);
		if(!marker) return 0;
		s->markers[i]=marker->object;
	}
	if(!box(s,0,0.34f,-0.19f,0.82f,0.18f,1.05f,blue,1) ||
		!box(s,0,0.49f,-0.18f,0.49f,0.15f,0.5f,(GRErgb24){30,43,58},1) ||
		!box(s,0,0.45f,0.28f,0.5f,0.09f,0.10f,(GRErgb24){255,229,145},1)) return 0;
	for(side=-1;side<=1;side+=2)
	{
		car_mesh* tire=addMesh(s,YMGRE_MeshGener_Cylinder(0.26f*U,0.16f*U,20,(GRErgb24){34,40,48},"drive-wheel",""),
			YMPME_Vec3(side*0.52f,0.26f,0),1,side<0?1:2,0);
		car_mesh* hub=addMesh(s,YMGRE_MeshGener_Cylinder(0.15f*U,0.175f*U,12,side<0?(GRErgb24){88,201,236}:(GRErgb24){247,174,60},"hub",""),
			YMPME_Vec3(side*0.52f,0.26f,0),1,side<0?1:2,0);
		if(!tire||!hub) return 0;
		/* 非对称轮毂色块，使自转可见。 */
		for(i=0;i<hub->object->polygonNum;++i)
			if(i%8<4) hub->object->polygonList[i].planeColor=(GRErgb24){222,237,242};
	}
	if(!addMesh(s,YMGRE_MeshGener_Cylinder(0.045f*U,0.16f*U,12,(GRErgb24){196,206,215},"caster-pivot",""),YMPME_Vec3(0,0.25f,-0.57f),1,0,0)) return 0;
	{
		car_mesh* fork=box(s,0,0.235f,-0.07f,0.20f,0.045f,0.22f,(GRErgb24){190,205,216},1);
		if(!fork) return 0;
		fork->caster=1;
		for(side=-1;side<=1;side+=2)
		{
			car_mesh* leg=box(s,side*0.085f,0.18f,-0.14f,0.03f,0.13f,0.06f,(GRErgb24){195,210,222},1);
			if(!leg) return 0;
			leg->caster=1;
		}
		if(!addMesh(s,YMGRE_MeshGener_Cylinder(0.13f*U,0.12f*U,16,(GRErgb24){242,179,70},"caster-wheel",""),
			YMPME_Vec3(0,0.13f,-0.14f),1,3,1)) return 0;
	}
	s->ambient=YMGRE_Creat_Light(0,GRE_GlobalLight,(GRErgb24){255,255,255},0.45f);
	s->keyLight=YMGRE_Creat_Light(1,GRE_PointLight,(GRErgb24){255,246,225},0.85f);
	if(!s->ambient||!s->keyLight) return 0;
	s->keyLight->pos=(gre_fvector4d){-180,350,-200,1};
	s->keyLight->proper.kc0=1;
	s->keyLight->proper.kc1=s->keyLight->proper.kc2=0;
	YMGRE_List_Append(&s->lights,sizeof(*s->ambient),s->ambient);
	YMGRE_List_Append(&s->lights,sizeof(*s->keyLight),s->keyLight);
	s->camera=YMGRE_Creat_Camera(0,800,520,38,38,28,28);
	s->workspace=YMGRE_Creat_RenderWorkspace();
	if(!s->camera||!s->workspace) return 0;
	YMGRE_Camera_Frustum_Init(s->camera,1,1500);
	s->camera->wireFrame=GRE_Render_Solid;
	return bindCollisionMeshes(s);
}
void CarScene_Reset(car_scene* s)
{
	int i;
	YMPME_Drive_Reset(&s->car);
	YMPME_Body_SetPosition(s->proxy,YMPME_Vec3(0,0,0));
	s->proxy->state.velocity=YMPME_Vec3(0,0,0);
	s->world.stepCount=0;
	s->targets=s->hits=s->nearObstacle=0;
	s->elapsed=0;
	for(i=0;i<CAR_TARGET_COUNT;++i) s->markers[i]->isVisible=1;
	memset(&s->world.collisionStats,0,sizeof(s->world.collisionStats));
	syncColliders(s);
}

void CarScene_SetMeshPrecision(car_scene* s,int enabled)
{
	s->world.config.meshPrecision=enabled!=0;
	/* AABB 模式可能扩大占用区域，切换时回到安全起点，避免模式切换后卡进障碍。 */
	CarScene_Reset(s);
}
int CarScene_Step(car_scene* s,float left,float right,int brake,float dt)
{
	pme_drive old=s->car,candidate;
	int i,hit;
	if(brake) s->car.left=s->car.right=0;
	if(YMPME_Drive_Step(&s->car,brake?0:left,brake?0:right,dt)!=PME_OK) return 0;
	candidate=s->car;
	if(YMPME_World_Step(&s->world,dt)!=PME_OK) return 0;
	syncColliders(s);
	hit=overlapsWorld(s);
	if(hit)
	{
		float low=0,high=1;
		/* 平面装配体整体回退位置和朝向，零件不能被单独推出而散架。
		 * 这是离散步的安全回退，不是连续碰撞检测；仍需小固定步长。
		 */
		for(i=0;i<8;++i)
		{
			float t=(low+high)*0.5f;
			s->car=blendPose(&old,&candidate,t);
			syncColliders(s);
			if(overlapsWorld(s)) high=t; else low=t;
		}
		s->car=blendPose(&old,&candidate,low);
		syncColliders(s);
	}
	YMPME_Body_SetPosition(s->proxy,YMPME_Vec3(s->car.x,0,s->car.z));
	if(hit&&!s->nearObstacle) ++s->hits;
	s->nearObstacle=hit;
	for(i=0;i<CAR_TARGET_COUNT;++i)
	{
		float x=s->car.x-checkpoints[i][0],z=s->car.z-checkpoints[i][1];
		if(s->markers[i]->isVisible && x*x+z*z<0.6f*0.6f)
		{
			s->markers[i]->isVisible=0;
			++s->targets;
		}
	}
	s->elapsed+=dt;
	return 1;
}
void CarScene_Render(car_scene* s,int cameraMode)
{
	int i;
	gre_fvector4d eye,target;
	pme_vec3 carPos=YMPME_Vec3(s->car.x,0,s->car.z);
	for(i=0;i<s->meshCount;++i)
	{
		car_mesh* m=&s->meshes[i];
		pme_vec3 position;
		float yaw,spin;
		if(!m->moving) continue;
		partPose(s,m,&position,&yaw,&spin);
		transform(m,position,yaw,spin);
	}
	if(cameraMode)
	{
		eye=(gre_fvector4d){0,410,-300,1};
		target=(gre_fvector4d){0,0,0,1};
	}
	else
	{
		eye=(gre_fvector4d){carPos.x*U+65,90,carPos.z*U-105,1};
		target=(gre_fvector4d){carPos.x*U,8,carPos.z*U,1};
	}
	YMGRE_UVNCamera_PositionInit(s->camera,&eye,&target,NULL,0);
	YMGRE_Camera_TanglePipline_RenderingWithWorkspace(s->camera,&s->lights,&s->objects,&s->materials,s->workspace);
}
static void keepObject(void* data) { (void)data; }
void CarScene_Destroy(car_scene* s)
{
	int i;
	if(s->workspace) YMGRE_Free_RenderWorkspace(s->workspace);
	if(s->camera) YMGRE_Free_Camera(s->camera);
	YMGRE_List_Clear(&s->objects,keepObject);
	YMGRE_List_Clear(&s->lights,keepObject);
	for(i=0;i<s->meshCount;++i)
	{
		free(s->meshes[i].local);
		free(s->meshes[i].normals);
		free(s->meshes[i].collisionVertices);
		free(s->meshes[i].collisionIndices);
		free(s->meshes[i].faceAxes);
		free(s->meshes[i].edgeAxes);
		if(s->meshes[i].object) YMGRE_Free_Object(s->meshes[i].object);
	}
	if(s->ambient) YMGRE_Free_Light(s->ambient);
	if(s->keyLight) YMGRE_Free_Light(s->keyLight);
	if(s->world.bodies) YMPME_World_Destroy(&s->world);
}
