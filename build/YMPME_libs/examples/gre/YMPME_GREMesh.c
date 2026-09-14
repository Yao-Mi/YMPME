#include "DEBUG/YMPME_Log.h"
#include "YMPME_GREMesh.h"
#include "CORE/YMPME_MathBase.h"
#include <stddef.h>
#include <string.h>
pme_result YMPME_GRE_Mesh_Bind(pme_gre_mesh* b,GRE_Object4d o,float units,
    pme_vec3* f,size_t nf,pme_vec3* e,size_t ne,gre_polygon4d* rf,size_t nr)
{
    if(!b||!o||!o->pointList||!o->pointList_||o->pointList==o->pointList_||
        !o->polygonList||!rf||rf==o->polygonList||o->pointNum<4||o->polygonNum<4||
        nr<(size_t)o->polygonNum||!isfinite(units)||units<=0||
        !isfinite(o->scale)||o->scale<=0) return YMPME_REPORT(PME_INVALID_ARGUMENT, "Invalid GRE source, work buffers, capacities or scale");
    for(int i=0;i<o->polygonNum;++i)
        if(o->polygonList[i].num!=3||!o->polygonList[i].index) return YMPME_REPORT(PME_INVALID_ARGUMENT, "GRE mesh must contain indexed triangles");
    pme_mesh_view view={o->pointList,(size_t)o->pointNum,sizeof(gre_vertex4d),
        offsetof(gre_vertex4d,pos),o->scale/units,NULL,o->polygonList,
        (size_t)o->polygonNum,sizeof(gre_polygon4d),offsetof(gre_polygon4d,index)};
    pme_gre_mesh out={0};
    pme_result result=YMPME_ConvexMesh_InitView(&out.mesh,&view,f,nf,e,ne);
    if(result!=PME_OK) return result;
    out.source=o; out.renderFaces=rf; out.workVertices=o->pointList_;
    out.unitsPerMeter=units; out.sourceScale=o->scale;
    *b=out; return PME_OK;
}
pme_result YMPME_GRE_Mesh_PrepareRender(pme_gre_mesh* b,pme_vec3 position,pme_rotation rotation)
{
    if(!b||!b->source||!YMPME_VecFinite(position)||!YMPME_Rotation_IsValid(rotation))
        return YMPME_REPORT(PME_INVALID_ARGUMENT, "Invalid shared binding, position or rotation");
    GRE_Object4d o=b->source;
    if(o->pointList!=b->mesh.view.vertices||o->polygonList!=b->mesh.view.triangles||
        o->pointList_!=b->workVertices||o->pointNum!=(int)b->mesh.vertexCount||
        o->polygonNum!=(int)b->mesh.triangleCount||o->scale!=b->sourceScale) return YMPME_REPORT(PME_INVALID_ARGUMENT, "GRE source buffers, counts or scale changed after binding");
    pme_vec3 translation=YMPME_VecScale(position,b->unitsPerMeter);
    for(int i=0;i<o->pointNum;++i) {
        pme_vec3 local=YMPME_ConvexMesh_Vertex(&b->mesh,(size_t)i);
        pme_vec3 world=YMPME_VecScale(YMPME_VecAdd(position,YMPME_Rotation_Apply(rotation,local)),b->unitsPerMeter);
        o->pointList_[i]=o->pointList[i];
        o->pointList_[i].pos=(gre_fvector4d){world.x,world.y,world.z,1};
    }
    for(int i=0;i<o->polygonNum;++i) {
        b->renderFaces[i]=o->polygonList[i];
        pme_vec3 a=YMPME_ConvexMesh_Vertex(&b->mesh,YMPME_ConvexMesh_Index(&b->mesh,3*(size_t)i));
        pme_vec3 c=YMPME_ConvexMesh_Vertex(&b->mesh,YMPME_ConvexMesh_Index(&b->mesh,3*(size_t)i+1));
        pme_vec3 d=YMPME_ConvexMesh_Vertex(&b->mesh,YMPME_ConvexMesh_Index(&b->mesh,3*(size_t)i+2));
        pme_vec3 n=YMPME_Rotation_Apply(rotation,YMPME_VecCross(YMPME_VecSub(c,a),YMPME_VecSub(d,a)));
        float length=sqrtf(YMPME_VecDot(n,n)); n=YMPME_VecScale(n,1/length);
        b->renderFaces[i].pN=(gre_fvector4d){n.x,n.y,n.z,0};
    }
    b->render=*o;
    b->render.pointList=o->pointList_; b->render.pointList_=NULL;
    b->render.pointList_wN=NULL; b->render.pointList_wN_=NULL;
    b->render.renderMode=GRE_RenderMode_Face;
    b->render.polygonList=b->renderFaces; b->render.nextObject=NULL;
    b->render.WorldCoordinate=(gre_fvector4d){translation.x,translation.y,translation.z,1};
    b->render.scale=1; b->render.boundType=GRE_Bounding_Sphere_R;
    float radius=0;
    for(size_t i=0;i<b->mesh.vertexCount;++i) {
        pme_vec3 v=YMPME_ConvexMesh_Vertex(&b->mesh,i);
        radius=fmaxf(radius,sqrtf(YMPME_VecDot(v,v))*b->unitsPerMeter);
    }
    b->render.BoundingSphereR=radius;
    return PME_OK;
}
void YMPME_GRE_Mesh_Unbind(pme_gre_mesh* b)
{
    if(b) memset(b,0,sizeof(*b));
}
