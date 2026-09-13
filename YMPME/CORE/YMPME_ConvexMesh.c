#include "YMPME_ConvexMesh.h"
#include "YMPME_MathBase.h"
#include <string.h>
pme_rotation YMPME_Rotation_Identity(void)
{
	pme_rotation r = {{{1,0,0},{0,1,0},{0,0,1}}};
	return r;
}
pme_vec3 YMPME_Rotation_Apply(pme_rotation r,pme_vec3 p)
{
	return YMPME_VecAdd(YMPME_VecAdd(YMPME_VecScale(r.axis[0],p.x),
		YMPME_VecScale(r.axis[1],p.y)),YMPME_VecScale(r.axis[2],p.z));
}
int YMPME_Rotation_IsValid(pme_rotation r)
{
	int i,j;
	for(i=0;i<3;++i)
	{
		if(!YMPME_VecFinite(r.axis[i]) || fabsf(YMPME_VecDot(r.axis[i],r.axis[i])-1)>1e-4f) return 0;
		for(j=i+1;j<3;++j) if(fabsf(YMPME_VecDot(r.axis[i],r.axis[j]))>1e-4f) return 0;
	}
	return YMPME_VecDot(YMPME_VecCross(r.axis[0],r.axis[1]),r.axis[2])>0.9999f;
}
static int addAxis(pme_vec3* axes,size_t* count,size_t capacity,pme_vec3 axis)
{
	size_t i;
	float length=sqrtf(YMPME_VecDot(axis,axis));
	if(length<1e-10f) return 0;
	axis=YMPME_VecScale(axis,1/length);
	for(i=0;i<*count;++i) if(fabsf(YMPME_VecDot(axis,axes[i]))>0.999999f) return 1;
	if(*count==capacity) return 0;
	axes[(*count)++]=axis;
	return 1;
}
static pme_result initMesh(pme_convex_mesh* mesh,const pme_vec3* v,size_t nv,
	const uint32_t* ix,const uint16_t* ix16,size_t nt,pme_vec3* faces,size_t nf,pme_vec3* edges,size_t ne)
{
	pme_convex_mesh m;
	pme_vec3 center={0,0,0};
	float scale,tolerance;
	double volume=0;
	size_t i,j,k;
	if(!mesh||!v||((ix!=NULL)==(ix16!=NULL))||!faces||!edges||nv<4||nv>UINT32_MAX||
        (ix16 && nv-1>UINT16_MAX)||nv>SIZE_MAX/sizeof(*v)||nt<4||
        nt>SIZE_MAX/(3*(ix16?sizeof(*ix16):sizeof(*ix)))) return PME_INVALID_ARGUMENT;
	memset(&m,0,sizeof(m));
    m.indices=ix; m.indices16=ix16;
	m.bounds.min=m.bounds.max=v[0];
	for(i=0;i<nv;++i)
	{
		if(!YMPME_VecFinite(v[i])) return PME_INVALID_ARGUMENT;
		center=YMPME_VecAdd(center,YMPME_VecScale(v[i],1.0f/(float)nv));
		m.bounds.min=YMPME_Vec3(fminf(m.bounds.min.x,v[i].x),fminf(m.bounds.min.y,v[i].y),fminf(m.bounds.min.z,v[i].z));
		m.bounds.max=YMPME_Vec3(fmaxf(m.bounds.max.x,v[i].x),fmaxf(m.bounds.max.y,v[i].y),fmaxf(m.bounds.max.z,v[i].z));
	}
	scale=sqrtf(YMPME_VecDot(YMPME_VecSub(m.bounds.max,m.bounds.min),YMPME_VecSub(m.bounds.max,m.bounds.min)));
	if(!isfinite(scale)||scale<1e-6f) return PME_INVALID_ARGUMENT;
	tolerance=scale*1e-5f;
	for(i=0;i<nt*3;++i) if(YMPME_ConvexMesh_Index(&m,i)>=nv) return PME_INVALID_ARGUMENT;
	for(i=0;i<nt;++i)
	{
		pme_vec3 a=v[YMPME_ConvexMesh_Index(&m,3*i)],b=v[YMPME_ConvexMesh_Index(&m,3*i+1)],c=v[YMPME_ConvexMesh_Index(&m,3*i+2)];
		pme_vec3 n=YMPME_VecCross(YMPME_VecSub(b,a),YMPME_VecSub(c,a));
		float length=sqrtf(YMPME_VecDot(n,n));
		if(!isfinite(length)||length<scale*scale*1e-9f) return PME_INVALID_ARGUMENT;
		n=YMPME_VecScale(n,1/length);
		if(YMPME_VecDot(n,YMPME_VecSub(center,a))>0) n=YMPME_VecScale(n,-1);
		for(j=0;j<nv;++j) if(YMPME_VecDot(n,YMPME_VecSub(v[j],a))>tolerance) return PME_INVALID_ARGUMENT;
		volume+=(double)length*fabsf(YMPME_VecDot(n,YMPME_VecSub(a,center)))/6;
		if(!addAxis(faces,&m.faceCount,nf,n)) return PME_CAPACITY_EXCEEDED;
		for(k=0;k<3;++k)
		{
			uint32_t start=YMPME_ConvexMesh_Index(&m,i*3+k),end=YMPME_ConvexMesh_Index(&m,i*3+(k+1)%3);
			size_t occurrences=0;
			for(j=0;j<nt*3;++j)
			{
				uint32_t x=YMPME_ConvexMesh_Index(&m,j),y=YMPME_ConvexMesh_Index(&m,(j/3)*3+(j+1)%3);
				if((x==start&&y==end)||(x==end&&y==start)) ++occurrences;
			}
			if(occurrences!=2) return PME_INVALID_ARGUMENT;
			if(!addAxis(edges,&m.edgeCount,ne,YMPME_VecSub(v[end],v[start]))) return PME_CAPACITY_EXCEEDED;
		}
	}
	if(volume<(double)scale*scale*scale*1e-8) return PME_INVALID_ARGUMENT;
	m.vertices=v; m.vertexCount=nv; m.indices=ix; m.triangleCount=nt;
	m.faceAxes=faces; m.edgeAxes=edges;
	*mesh=m;
	return PME_OK;
}

pme_result YMPME_ConvexMesh_Init(pme_convex_mesh* mesh,const pme_vec3* v,size_t nv,
    const uint32_t* ix,size_t nt,pme_vec3* faces,size_t nf,pme_vec3* edges,size_t ne)
{
    return initMesh(mesh,v,nv,ix,NULL,nt,faces,nf,edges,ne);
}
pme_result YMPME_ConvexMesh_Init16(pme_convex_mesh* mesh,const pme_vec3* v,size_t nv,
    const uint16_t* ix,size_t nt,pme_vec3* faces,size_t nf,pme_vec3* edges,size_t ne)
{
    return initMesh(mesh,v,nv,NULL,ix,nt,faces,nf,edges,ne);
}

pme_vec3 YMPME_Triangle_ClosestPoint(pme_vec3 p,pme_vec3 a,pme_vec3 b,pme_vec3 c)
{
	pme_vec3 ab=YMPME_VecSub(b,a),ac=YMPME_VecSub(c,a),ap=YMPME_VecSub(p,a),bp,cp;
	float d1=YMPME_VecDot(ab,ap),d2=YMPME_VecDot(ac,ap),d3,d4,d5,d6,va,vb,vc;
	if(d1<=0&&d2<=0) return a;
	bp=YMPME_VecSub(p,b); d3=YMPME_VecDot(ab,bp); d4=YMPME_VecDot(ac,bp);
	if(d3>=0&&d4<=d3) return b;
	vc=d1*d4-d3*d2;
	if(vc<=0&&d1>=0&&d3<=0) return YMPME_VecAdd(a,YMPME_VecScale(ab,d1/(d1-d3)));
	cp=YMPME_VecSub(p,c); d5=YMPME_VecDot(ab,cp); d6=YMPME_VecDot(ac,cp);
	if(d6>=0&&d5<=d6) return c;
	vb=d5*d2-d1*d6;
	if(vb<=0&&d2>=0&&d6<=0) return YMPME_VecAdd(a,YMPME_VecScale(ac,d2/(d2-d6)));
	va=d3*d6-d5*d4;
	if(va<=0&&(d4-d3)>=0&&(d5-d6)>=0)
		return YMPME_VecAdd(b,YMPME_VecScale(YMPME_VecSub(c,b),(d4-d3)/((d4-d3)+(d5-d6))));
	return YMPME_VecAdd(a,YMPME_VecAdd(YMPME_VecScale(ab,vb/(va+vb+vc)),YMPME_VecScale(ac,vc/(va+vb+vc))));
}
