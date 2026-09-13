/* 凸多面体 SAT：双方全部面法线 + 边叉积；球体补最近三角形特征方向。 */
#include "YMPME_MeshSAT.h"
#include "YMPME_MathBase.h"
#include <float.h>
static void project(const pme_body* b,pme_vec3 axis,float* low,float* high)
{
	const pme_body_desc* d=&b->state;
	float center=YMPME_VecDot(d->position,axis);
	if(d->shape==PME_SPHERE) { *low=center-d->radius; *high=center+d->radius; }
	else if(d->shape==PME_BOX)
	{
		float radius=fabsf(axis.x)*d->halfExtents.x+fabsf(axis.y)*d->halfExtents.y+fabsf(axis.z)*d->halfExtents.z;
		*low=center-radius; *high=center+radius;
	}
	else
	{
		size_t i;
		pme_vec3 local=YMPME_Vec3(YMPME_VecDot(axis,d->rotation.axis[0]),YMPME_VecDot(axis,d->rotation.axis[1]),YMPME_VecDot(axis,d->rotation.axis[2]));
		*low=FLT_MAX; *high=-FLT_MAX;
		for(i=0;i<d->mesh->vertexCount;++i)
		{
			float p=YMPME_VecDot(local,d->mesh->vertices[i])+center;
			*low=fminf(*low,p); *high=fmaxf(*high,p);
		}
	}
}
static int axisTest(const pme_body* a,const pme_body* b,pme_vec3 axis,pme_contact* best)
{
	float al,ah,bl,bh,depth,length=YMPME_VecDot(axis,axis);
	if(length<1e-12f) return 1;
	axis=YMPME_VecScale(axis,1/sqrtf(length));
	project(a,axis,&al,&ah); project(b,axis,&bl,&bh);
	if(ah<bl || bh<al) return 0;
	/* 采用完整分离距离，包含一个形状完全包住另一个的情况。 */
	depth=ah-bl;
	if(bh-al<depth) { depth=bh-al; axis=YMPME_VecScale(axis,-1); }
	if(depth<best->penetration) { best->penetration=depth; best->normal=axis; }
	return 1;
}
static size_t axisCount(const pme_body* b,int edge)
{
	if(b->state.shape==PME_SPHERE) return 0;
	if(b->state.shape==PME_BOX) return 3;
	return edge?b->state.mesh->edgeCount:b->state.mesh->faceCount;
}
static pme_vec3 getAxis(const pme_body* b,size_t i,int edge)
{
	if(b->state.shape==PME_BOX) return YMPME_Vec3(i==0,i==1,i==2);
	return YMPME_Rotation_Apply(b->state.rotation,edge?b->state.mesh->edgeAxes[i]:b->state.mesh->faceAxes[i]);
}
int YMPME_MeshSAT(const pme_body* a,const pme_body* b,pme_contact* contact)
{
	pme_contact best;
	size_t i,j;
	best.penetration=FLT_MAX; best.normal=YMPME_Vec3(1,0,0);
	for(i=0;i<axisCount(a,0);++i) if(!axisTest(a,b,getAxis(a,i,0),&best)) return 0;
	for(i=0;i<axisCount(b,0);++i) if(!axisTest(a,b,getAxis(b,i,0),&best)) return 0;
	for(i=0;i<axisCount(a,1);++i)
		for(j=0;j<axisCount(b,1);++j)
			if(!axisTest(a,b,YMPME_VecCross(getAxis(a,i,1),getAxis(b,j,1)),&best)) return 0;
	if(a->state.shape==PME_SPHERE || b->state.shape==PME_SPHERE)
	{
		const pme_body* sphere=a->state.shape==PME_SPHERE?a:b;
		const pme_body* mesh=sphere==a?b:a;
		const pme_convex_mesh* m=mesh->state.mesh;
		for(i=0;i<m->triangleCount;++i)
		{
			pme_vec3 v[3],q;
			for(j=0;j<3;++j) v[j]=YMPME_VecAdd(mesh->state.position,YMPME_Rotation_Apply(mesh->state.rotation,m->vertices[YMPME_ConvexMesh_Index(m,3*i+j)]));
			q=YMPME_Triangle_ClosestPoint(sphere->state.position,v[0],v[1],v[2]);
			if(!axisTest(a,b,YMPME_VecSub(q,sphere->state.position),&best)) return 0;
		}
	}
	if(contact) *contact=best;
	return 1;
}
