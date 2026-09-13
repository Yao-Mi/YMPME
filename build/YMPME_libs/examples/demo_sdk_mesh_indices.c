#include "YMPME.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stddef.h>
#define CHECK(x) do { if(!(x)) { fprintf(stderr,"line %d: %s\n",__LINE__,#x); exit(1); } } while(0)
static const pme_vec3 vertices[8]={{-.5f,-.5f,-.5f},{.5f,-.5f,-.5f},{.5f,.5f,-.5f},{-.5f,.5f,-.5f},
 {-.5f,-.5f,.5f},{.5f,-.5f,.5f},{.5f,.5f,.5f},{-.5f,.5f,.5f}};
static const PME_Index indices[36]={0,2,1,0,3,2,4,5,6,4,6,7,0,1,5,0,5,4,3,7,6,3,6,2,0,4,7,0,7,3,1,2,6,1,6,5};
typedef struct vertex_record_ { unsigned tag; float xyz[3]; float uv[2]; } vertex_record;
typedef struct face_record_ { float color[3]; const PME_Index* index; } face_record;
static void collision(const pme_convex_mesh* a,const pme_convex_mesh* b)
{
    pme_body ba,bb; pme_contact c;
    pme_body_desc d=YMPME_Body_DefaultDesc(); d.shape=PME_CONVEX_MESH; d.mesh=a;
    CHECK(YMPME_Body_Init(&ba,&d)==PME_OK);
    d.mesh=b; d.position.x=.8f; CHECK(YMPME_Body_Init(&bb,&d)==PME_OK);
    CHECK(YMPME_Collide(&ba,&bb,&c)); CHECK(fabsf(c.penetration-.2f)<1e-5f);
    d.shape=PME_SPHERE; d.radius=.25f; d.position=YMPME_Vec3(.6f,0,0);
    CHECK(YMPME_Body_Init(&bb,&d)==PME_OK); CHECK(YMPME_Collide(&ba,&bb,&c));
    CHECK(fabsf(c.penetration-.15f)<1e-5f);
}
int main(void)
{
    CHECK(sizeof(PME_Index)*8==YMPME_INDEX_BITS);
    vertex_record v[8]; face_record rows[12];
    for(int i=0;i<8;++i) { v[i].tag=(unsigned)i; v[i].xyz[0]=vertices[i].x*100;
        v[i].xyz[1]=vertices[i].y*100; v[i].xyz[2]=vertices[i].z*100; v[i].uv[0]=v[i].uv[1]=.25f; }
    for(int i=0;i<12;++i) { memset(&rows[i],0,sizeof(rows[i])); rows[i].index=&indices[3*i]; }
    pme_vec3 f[3][12],e[3][36]; pme_convex_mesh compact,shared,flat;
    CHECK(YMPME_ConvexMesh_Init(&compact,vertices,8,indices,12,f[0],12,e[0],36)==PME_OK);
    pme_mesh_view view={v,8,sizeof(v[0]),offsetof(vertex_record,xyz),.01f,NULL,rows,12,sizeof(rows[0]),offsetof(face_record,index)};
    CHECK(YMPME_ConvexMesh_InitView(&shared,&view,f[1],12,e[1],36)==PME_OK);
    CHECK(shared.view.vertices==v && shared.view.triangles==rows && shared.vertices==NULL && shared.indices==NULL);
    for(size_t i=0;i<36;++i) CHECK(YMPME_ConvexMesh_Index(&shared,i)==indices[i]);
    collision(&compact,&shared); collision(&shared,&compact); collision(&shared,&shared);
    view.indices=indices; view.triangles=NULL;
    CHECK(YMPME_ConvexMesh_InitView(&flat,&view,f[2],12,e[2],36)==PME_OK); collision(&flat,&shared);
    pme_convex_mesh saved=shared,failed=saved; pme_mesh_view bad=view;
    bad.vertexStride=1;
    CHECK(YMPME_ConvexMesh_InitView(&failed,&bad,f[1],12,e[1],36)==PME_INVALID_ARGUMENT);
    CHECK(!memcmp(&failed,&saved,sizeof(saved)));
    bad=view; bad.positionOffset=bad.vertexStride;
    CHECK(YMPME_ConvexMesh_InitView(&failed,&bad,f[1],12,e[1],36)==PME_INVALID_ARGUMENT);
    bad=view; bad.triangles=rows;
    CHECK(YMPME_ConvexMesh_InitView(&failed,&bad,f[1],12,e[1],36)==PME_INVALID_ARGUMENT);
    bad=view; bad.vertexScale=NAN;
    CHECK(YMPME_ConvexMesh_InitView(&failed,&bad,f[1],12,e[1],36)==PME_INVALID_ARGUMENT);
    bad=shared.view; rows[0].index=NULL;
    CHECK(YMPME_ConvexMesh_InitView(&failed,&bad,f[1],12,e[1],36)==PME_INVALID_ARGUMENT);
    rows[0].index=indices;
    PME_Index wrong[36]; memcpy(wrong,indices,sizeof(wrong)); wrong[0]=8;
    bad=view; bad.indices=wrong;
    CHECK(YMPME_ConvexMesh_InitView(&failed,&bad,f[1],12,e[1],36)==PME_INVALID_ARGUMENT);
    CHECK(YMPME_ConvexMesh_InitView(&shared,&saved.view,f[1],12,e[1],36)==PME_OK);
#if PME_ENABLE_QUERY
    pme_world world; pme_body storage[1]; CHECK(YMPME_World_Init(&world,storage,1,NULL)==PME_OK);
    pme_body_desc d=YMPME_Body_DefaultDesc(); d.type=PME_STATIC; d.shape=PME_CONVEX_MESH; d.mesh=&shared;
    CHECK(YMPME_World_AddBody(&world,&d));
    pme_world_query q={&world,YMPME_Query_DefaultFilter()};
    pme_ray ray={{0,0,-2},{0,0,1},5}; pme_ray_hit hit;
    CHECK(YMPME_Query_WorldRaycast(&q,&ray,&hit)==PME_OK && hit.hit && fabsf(hit.distance-1.5f)<1e-5f);
    YMPME_World_Destroy(&world);
#endif
#if PME_ENABLE_DYNAMICS
    pme_rigid_body bodies[2];
    for(int i=0;i<2;++i) CHECK(YMPME_Rigid_Init(&bodies[i],(float)i,YMPME_Vec3(.1f,.1f,.1f),
        YMPME_Vec3(0,i?1.5f:0,0),YMPME_Quaternion_Identity())==PME_OK);
    pme_collider shapes[2]={YMPME_Dynamics_Collider(0,PME_COLLIDER_MESH),YMPME_Dynamics_Collider(1,PME_COLLIDER_MESH)};
    shapes[0].mesh=&shared; shapes[1].mesh=&compact;
    pme_dynamic_contact ct[32]; pme_dynamics w=YMPME_Dynamics_Default();
    w.bodies=bodies; w.bodyCount=2; w.colliders=shapes; w.colliderCount=2; w.contacts=ct; w.contactCapacity=32;
    for(int i=0;i<480;++i) CHECK(YMPME_Dynamics_Step(&w,1.f/240,w.time+1./240)==PME_OK);
    CHECK(fabsf(bodies[1].position.y-1)<.01f && w.contactCount>0);
    bodies[1].position=YMPME_Vec3(-2,0,0); bodies[1].velocity=YMPME_Vec3(100,0,0);
    shapes[1].type=PME_COLLIDER_SPHERE; shapes[1].radius=.1f;
    w.gravity=YMPME_Vec3(0,0,0); uint8_t enabled[2]={0,1}; pme_ccd_event ev[8]; pme_ccd ccd;
    CHECK(YMPME_Dynamics_CCDInit(&ccd,enabled,2,ev,8)==PME_OK); w.ccd=&ccd;
    CHECK(YMPME_Dynamics_Step(&w,.03f,w.time+.03)==PME_OK);
    CHECK(ccd.count>0 && bodies[1].position.x<-.59f);
#endif
    /* Exercise the largest 16-bit index and a genuinely >16-bit index in the 32-bit build. */
    size_t top=YMPME_INDEX_BITS==16?65535u:65536u;
    pme_vec3* large=calloc(top+1,sizeof(*large)); CHECK(large);
    large[1]=YMPME_Vec3(1,0,0); large[2]=YMPME_Vec3(0,1,0); large[top]=YMPME_Vec3(0,0,1);
    PME_Index tetra[12]={0,2,1,0,1,(PME_Index)top,0,(PME_Index)top,2,1,2,(PME_Index)top};
    pme_convex_mesh boundary; pme_vec3 bf[4],be[12];
    CHECK(YMPME_ConvexMesh_Init(&boundary,large,top+1,tetra,4,bf,4,be,12)==PME_OK);
    CHECK(YMPME_ConvexMesh_Index(&boundary,5)==top);
#if YMPME_INDEX_BITS==16
    CHECK(YMPME_ConvexMesh_Init(&failed,vertices,65537,indices,12,f[0],12,e[0],36)==PME_INVALID_ARGUMENT);
#endif
    free(large);
    printf("PASS index%d: borrowed vertices/face indices, SAT, query, dynamics, CCD and index boundaries\n",YMPME_INDEX_BITS);
    return 0;
}
