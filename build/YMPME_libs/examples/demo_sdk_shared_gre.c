#include "YMPME.h"
#include "YMPME_GREMesh.h"
#include "YMGRE_Creat.h"
#include "YMGRE_Free.h"
#include "YMGRE_Mem.h"
#include "YMGRE_Rendering_Pipeline.h"
#include "YMGRE_Camera.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define CHECK(x) do { if(!(x)) { fprintf(stderr,"line %d: %s\n",__LINE__,#x); exit(1); } } while(0)
static void keep(void* p) { (void)p; }
int main(void)
{
    static const float xyz[8][3]={{-50,-50,-50},{50,-50,-50},{50,50,-50},{-50,50,-50},
        {-50,-50,50},{50,-50,50},{50,50,50},{-50,50,50}};
    static const GRE_Index ix[36]={0,2,1,0,3,2,4,5,6,4,6,7,0,1,5,0,5,4,3,7,6,3,6,2,0,4,7,0,7,3,1,2,6,1,6,5};
    GRE_Object4d source=YMGRE_Creat_Object(8,12,"shared geometry",""); CHECK(source);
    for(int i=0;i<8;++i) source->pointList[i].pos=(gre_fvector4d){xyz[i][0],xyz[i][1],xyz[i][2],1};
    for(int i=0;i<12;++i) {
        source->polygonList[i].num=3;
        source->polygonList[i].index=GRE_PolyIndex_Malloc(3*sizeof(GRE_Index));
        CHECK(source->polygonList[i].index);
        memcpy(source->polygonList[i].index,ix+3*i,3*sizeof(GRE_Index));
        source->polygonList[i].planeColor=(GRErgb24){50,180,220};
    }
    gre_vertex4d original[8]; gre_polygon4d originalFaces[12];
    memcpy(original,source->pointList,sizeof(original)); memcpy(originalFaces,source->polygonList,sizeof(originalFaces));
    pme_vec3 f[12],e[36]; gre_polygon4d rf[12]; pme_gre_mesh binding={0};
    CHECK(YMPME_GRE_Mesh_Bind(&binding,source,100,f,12,e,36,rf,12)==PME_OK);
    CHECK(binding.mesh.view.vertices==source->pointList && binding.mesh.view.triangles==source->polygonList);
    CHECK(binding.mesh.vertices==NULL && binding.mesh.indices==NULL);
    CHECK(sizeof(PME_Index)==sizeof(GRE_Index));
    pme_body meshBody,sphere; pme_body_desc d=YMPME_Body_DefaultDesc();
    d.shape=PME_CONVEX_MESH; d.mesh=&binding.mesh; CHECK(YMPME_Body_Init(&meshBody,&d)==PME_OK);
    d.shape=PME_SPHERE; d.radius=.25f; d.position.x=.6f;
    CHECK(YMPME_Body_Init(&sphere,&d)==PME_OK); pme_contact c;
    CHECK(YMPME_Collide(&meshBody,&sphere,&c) && fabsf(c.penetration-.15f)<1e-5f);
    pme_rotation rotation={{{.8f,0,-.6f},{0,1,0},{.6f,0,.8f}}};
    CHECK(YMPME_GRE_Mesh_PrepareRender(&binding,YMPME_Vec3(.15f,0,3),rotation)==PME_OK);
    for(int i=0;i<12;++i) CHECK(binding.render.polygonList[i].index==source->polygonList[i].index);
    CHECK(binding.render.pointList==source->pointList_);
    pme_vec3 expected=YMPME_VecScale(YMPME_VecAdd(YMPME_Vec3(.15f,0,3),
        YMPME_Rotation_Apply(rotation,YMPME_ConvexMesh_Vertex(&binding.mesh,0))),100);
    CHECK(fabsf(binding.render.pointList[0].pos.x-expected.x)<1e-4f);
    gre_list objects={0},lights={0},materials={0};
    GRE_Light4d ambient=YMGRE_Creat_Light(0,GRE_GlobalLight,(GRErgb24){255,255,255},1); CHECK(ambient);
    YMGRE_List_Append(&objects,sizeof(binding.render),&binding.render);
    YMGRE_List_Append(&lights,sizeof(*ambient),ambient);
    GRE_Camera4d camera=YMGRE_Creat_Camera(0,160,160,35,35,35,35); CHECK(camera);
    YMGRE_Camera_Frustum_Init(camera,1,1000);
    gre_fvector4d eye={0,0,0,1},target={0,0,300,1};
    YMGRE_UVNCamera_PositionInit(camera,&eye,&target,NULL,0);
    GRE_RenderWorkspace work=YMGRE_Creat_RenderWorkspace(); CHECK(work);
    YMGRE_Camera_TanglePipline_RenderingWithWorkspace(camera,&lights,&objects,&materials,work);
    unsigned visible=0; for(int i=0;i<160*160;++i) if(camera->img.zbuff[i]<1000) ++visible;
    CHECK(visible>100);
    CHECK(!memcmp(original,source->pointList,sizeof(original)));
    CHECK(!memcmp(originalFaces,source->polygonList,sizeof(originalFaces)));
    for(int i=0;i<12;++i) CHECK(!memcmp(source->polygonList[i].index,ix+3*i,3*sizeof(GRE_Index)));
    CHECK(YMPME_Collide(&meshBody,&sphere,&c) && fabsf(c.penetration-.15f)<1e-5f);
    source->scale=2; CHECK(YMPME_GRE_Mesh_PrepareRender(&binding,YMPME_Vec3(0,0,0),rotation)==PME_INVALID_ARGUMENT); source->scale=1;
    /* Detach render/physics users, then unbind, then release the GRE owner. */
    YMGRE_List_Clear(&objects,keep); meshBody.state.mesh=NULL;
    YMPME_GRE_Mesh_Unbind(&binding); CHECK(binding.source==NULL);
    CHECK(YMPME_GRE_Mesh_PrepareRender(&binding,YMPME_Vec3(0,0,0),rotation)==PME_INVALID_ARGUMENT);
    YMGRE_Free_RenderWorkspace(work); YMGRE_Free_Camera(camera);
    YMGRE_List_Clear(&lights,YMGRE_Free_Light); YMGRE_Free_Object(source);
    printf("PASS shared GRE/PME index%d: zero geometry copies, rendered %u pixels, source unchanged, safe detach\n",YMPME_INDEX_BITS,visible);
    return 0;
}
