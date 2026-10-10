#include "YMPME.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define CHECK(x) do { if (!(x)) { fprintf(stderr,"%s:%d: %s\n",__FILE__,__LINE__,#x); return 1; } } while(0)
#define NEAR(a,b,e) CHECK(fabsf((a)-(b))<(e))
static size_t allocations;
void* PME_malloc0(size_t n) { ++allocations; return malloc(n); }
void PME_free0(void* p) { free(p); }
static const pme_vec3 cube[]={{-0.5f,-0.5f,-0.5f},{0.5f,-0.5f,-0.5f},{0.5f,0.5f,-0.5f},{-0.5f,0.5f,-0.5f},
 {-0.5f,-0.5f,0.5f},{0.5f,-0.5f,0.5f},{0.5f,0.5f,0.5f},{-0.5f,0.5f,0.5f}};
static const PME_Index cubeIx[]={0,2,1,0,3,2,4,5,6,4,6,7,0,1,5,0,5,4,3,7,6,3,6,2,0,4,7,0,7,3,1,2,6,1,6,5};
static const pme_vec3 tetra[]={{0,0,0},{1,0,0},{0,1,0},{0,0,1}};
static const PME_Index tetraIx[]={0,2,1,0,1,3,0,3,2,1,2,3};
static pme_rotation yaw(float angle)
{
 pme_rotation r={{{cosf(angle),0,-sinf(angle)},{0,1,0},{sinf(angle),0,cosf(angle)}}}; return r;
}
int main(void)
{
 pme_convex_mesh cm,tm,failed={0};
 pme_vec3 cf[12],ce[36],tf[4],te[12];
 pme_body a,b;
 pme_body_desc d=YMPME_Body_DefaultDesc();
 pme_contact c,reverse,sentinel;
 pme_collision_stats stats={0};
 pme_collision_options options={1,NULL,&stats};
 pme_world w; pme_body bodies[4];
 size_t start;
 int i;
 CHECK(YMPME_ConvexMesh_Init(&cm,cube,8,cubeIx,12,cf,12,ce,36)==PME_OK);
 CHECK(YMPME_ConvexMesh_Init(&tm,tetra,4,tetraIx,4,tf,4,te,12)==PME_OK);
 CHECK(cm.faceCount==3);
 CHECK(YMPME_ConvexMesh_Init(&failed,cube,8,cubeIx,11,cf,12,ce,36)==PME_INVALID_ARGUMENT);
 CHECK(!failed.vertices);
 CHECK(YMPME_ConvexMesh_Init(&failed,cube,8,cubeIx,12,cf,0,ce,36)==PME_CAPACITY_EXCEEDED);
 {
  pme_vec3 concave[8]; PME_Index invalid[36];
  memcpy(concave,cube,sizeof(cube)); concave[6]=(pme_vec3){0,0,0};
  CHECK(YMPME_ConvexMesh_Init(&failed,concave,8,cubeIx,12,cf,12,ce,36)==PME_INVALID_ARGUMENT);
  memcpy(invalid,cubeIx,sizeof(cubeIx)); invalid[0]=80;
  CHECK(YMPME_ConvexMesh_Init(&failed,cube,8,invalid,12,cf,12,ce,36)==PME_INVALID_ARGUMENT);
 }
 /* 失败调用可覆盖外部缓存，重新初始化后才使用 Mesh。 */
 CHECK(YMPME_ConvexMesh_Init(&cm,cube,8,cubeIx,12,cf,12,ce,36)==PME_OK);
 d.shape=PME_CONVEX_MESH; d.mesh=&tm;
 CHECK(YMPME_Body_Init(&a,&d)==PME_OK);
 d.shape=PME_BOX; d.halfExtents=(pme_vec3){0.05f,0.05f,0.05f}; d.position=(pme_vec3){0.8f,0.8f,0.8f};
 CHECK(YMPME_Body_Init(&b,&d)==PME_OK);
 sentinel.normal=(pme_vec3){7,8,9}; sentinel.penetration=10; c=sentinel;
 CHECK(!YMPME_CollideWithOptions(&a,&b,&options,&c));
 CHECK(memcmp(&c,&sentinel,sizeof(c))==0);
 CHECK(stats.aabbTests==1 && stats.aabbRejected==0 && stats.meshTests==1);
 options.meshPrecision=0;
 CHECK(YMPME_CollideWithOptions(&a,&b,&options,&c));
 CHECK(stats.meshTests==1);
 options.meshPrecision=1;
 b.state.position.x=4;
 CHECK(!YMPME_CollideWithOptions(&a,&b,&options,&c));
 CHECK(stats.aabbRejected==1 && stats.meshTests==1);
 /* Mesh/Mesh：两个四面体的 AABB 重叠，实际凸体分离。 */
 d.shape=PME_CONVEX_MESH; d.mesh=&tm; d.position=(pme_vec3){0.6f,0.6f,0.6f};
 CHECK(YMPME_Body_Init(&b,&d)==PME_OK);
 CHECK(!YMPME_Collide(&a,&b,&c));
 d.mesh=&cm; d.position=(pme_vec3){0,0,0};
 CHECK(YMPME_Body_Init(&a,&d)==PME_OK);
 d.position.x=0.8f;
 CHECK(YMPME_Body_Init(&b,&d)==PME_OK);
 CHECK(YMPME_Collide(&a,&b,&c));
 NEAR(c.penetration,0.2f,1e-5f); NEAR(c.normal.x,1,1e-5f);
 CHECK(YMPME_Collide(&b,&a,&reverse));
 NEAR(YMPME_VecDot(c.normal,reverse.normal),-1,1e-5f);
 /* 真正旋转后的凸体，不是旋转后 AABB 的替代。 */
 CHECK(YMPME_Body_SetRotation(&a,yaw(0.785398163f))==PME_OK);
 d.shape=PME_BOX; d.position=(pme_vec3){0.65f,0,0.65f}; d.halfExtents=(pme_vec3){0.02f,0.02f,0.02f};
 CHECK(YMPME_Body_Init(&b,&d)==PME_OK);
 options.meshPrecision=0; CHECK(YMPME_CollideWithOptions(&a,&b,&options,&c));
 options.meshPrecision=1; CHECK(!YMPME_CollideWithOptions(&a,&b,&options,&c));
 CHECK(YMPME_Body_SetRotation(&a,YMPME_Rotation_Identity())==PME_OK);
 /* Sphere/Mesh 边角以及完全包含的最小分离深度。 */
 d.shape=PME_SPHERE; d.radius=0.4f; d.position=(pme_vec3){0.8f,0.8f,0};
 CHECK(YMPME_Body_Init(&b,&d)==PME_OK);
 CHECK(!YMPME_Collide(&a,&b,&c));
 b.state.position=(pme_vec3){0.78f,0.78f,0};
 CHECK(YMPME_Collide(&a,&b,&c));
 NEAR(c.penetration,0.4f-sqrtf(2*0.28f*0.28f),1e-5f);
 b.state.position=(pme_vec3){0,0,0}; b.state.radius=0.1f;
 CHECK(YMPME_Collide(&a,&b,&c)); NEAR(c.penetration,0.6f,1e-5f);
 /* 过滤必须在 AABB 之前；矩阵双向、同组、自装配体、层掩码。 */
 CHECK(YMPME_World_Init(&w,bodies,4,NULL)==PME_OK);
 a.state.collisionGroup=1; b.state.collisionGroup=2;
 CHECK(YMPME_World_SetGroupCollision(&w,1,2,0)==PME_OK);
 CHECK(!YMPME_World_Collide(&w,&a,&b,&c));
 CHECK(!YMPME_World_Collide(&w,&b,&a,&c));
 CHECK(w.collisionStats.filteredPairs==2 && w.collisionStats.aabbTests==0);
 CHECK(YMPME_World_SetGroupCollision(&w,1,2,1)==PME_OK);
 CHECK(YMPME_World_Collide(&w,&a,&b,&c));
 b.state.collisionGroup=1;
 CHECK(YMPME_World_SetGroupCollision(&w,1,1,0)==PME_OK);
 CHECK(!YMPME_World_Collide(&w,&a,&b,&c));
 CHECK(YMPME_World_SetGroupCollision(&w,1,1,1)==PME_OK);
 a.state.ignoreAssembly=b.state.ignoreAssembly=42;
 CHECK(!YMPME_Collide(&a,&b,&c));
 b.state.ignoreAssembly=43; CHECK(YMPME_Collide(&a,&b,&c));
 b.state.mask=0; CHECK(!YMPME_Collide(&a,&b,&c));
 CHECK(YMPME_World_SetGroupCollision(&w,32,0,0)==PME_INVALID_ARGUMENT);
 /* 真实 World 求解：凸 Mesh 落在盒地面上，且步进不分配。 */
 d=YMPME_Body_DefaultDesc(); d.type=PME_STATIC; d.shape=PME_BOX;
 d.position.y=-0.5f; d.halfExtents=(pme_vec3){10,0.5f,10}; CHECK(YMPME_World_AddBody(&w,&d));
 d=YMPME_Body_DefaultDesc(); d.shape=PME_CONVEX_MESH; d.mesh=&cm; d.position.y=2;
 { PME_Body falling=YMPME_World_AddBody(&w,&d); CHECK(falling);
  start=allocations;
  for(i=0;i<600;++i) CHECK(YMPME_World_Step(&w,w.config.fixedStep)==PME_OK);
  NEAR(falling->state.position.y,0.5f,0.001f);
  NEAR(falling->state.velocity.y,0,1e-5f);
  CHECK(allocations==start && w.collisionStats.meshTests>0);
 }
 YMPME_World_Destroy(&w);
 puts("PASS convex mesh: validation, AABB gate, SAT, sphere, containment, groups, solver, zero allocation");
 return 0;
}
