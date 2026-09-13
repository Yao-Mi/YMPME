#include "YMPME_Collision.h"
#include "YMPME_MathBase.h"
#include "YMPME_MeshSAT.h"
static int sphereSphere(const pme_body_desc* a, const pme_body_desc* b, pme_contact* c)
{
	pme_vec3 delta = YMPME_VecSub(b->position, a->position);
	float d2 = YMPME_VecDot(delta, delta), r = a->radius + b->radius;
	float distance;
	if (d2 > r*r) return 0;
	distance = sqrtf(d2);
	c->normal = distance > 1e-8f ? YMPME_VecScale(delta, 1.0f/distance) : YMPME_Vec3(1, 0, 0);
	c->penetration = r-distance;
	return 1;
}
static int boxBox(const pme_body_desc* a, const pme_body_desc* b, pme_contact* c)
{
	pme_vec3 d = YMPME_VecSub(b->position, a->position);
	pme_vec3 h = YMPME_VecAdd(a->halfExtents, b->halfExtents);
	float x = h.x-fabsf(d.x), y = h.y-fabsf(d.y), z = h.z-fabsf(d.z);
	if (x < 0 || y < 0 || z < 0) return 0;
	c->penetration = x;
	c->normal = YMPME_Vec3(d.x < 0 ? -1.0f : 1.0f, 0, 0);
	if (y < c->penetration)
	{
		c->penetration = y;
		c->normal = YMPME_Vec3(0, d.y < 0 ? -1.0f : 1.0f, 0);
	}
	if (z < c->penetration)
	{
		c->penetration = z;
		c->normal = YMPME_Vec3(0, 0, d.z < 0 ? -1.0f : 1.0f);
	}
	return 1;
}
static int sphereBox(const pme_body_desc* s, const pme_body_desc* b, pme_contact* c)
{
	pme_vec3 p = YMPME_VecSub(s->position, b->position), h = b->halfExtents;
	pme_vec3 q = YMPME_Vec3(YMPME_Clamp(p.x, -h.x, h.x),
		YMPME_Clamp(p.y, -h.y, h.y), YMPME_Clamp(p.z, -h.z, h.z));
	pme_vec3 delta = YMPME_VecSub(q, p);
	float d2 = YMPME_VecDot(delta, delta);
	if (d2 > s->radius*s->radius) return 0;
	if (d2 > 0)
	{
		float distance = sqrtf(d2);
		c->normal = YMPME_VecScale(delta, 1.0f/distance);
		c->penetration = s->radius-distance;
	}
	else
	{
		/* 球心在盒内或表面：选最近出口；A 沿负法线移出。 */
		float x = h.x-fabsf(p.x), y = h.y-fabsf(p.y), z = h.z-fabsf(p.z);
		c->normal = YMPME_Vec3(p.x < 0 ? 1.0f : -1.0f, 0, 0);
		c->penetration = x+s->radius;
		if (y < x)
		{
			x = y;
			c->normal = YMPME_Vec3(0, p.y < 0 ? 1.0f : -1.0f, 0);
			c->penetration = y+s->radius;
		}
		if (z < x)
		{
			c->normal = YMPME_Vec3(0, 0, p.z < 0 ? 1.0f : -1.0f);
			c->penetration = z+s->radius;
		}
	}
	return 1;
}

int YMPME_ShouldCollide(const pme_body* a,const pme_body* b,const pme_collision_options* o)
{
	if(!a||!b||a==b||!a->active||!b->active||a->state.collisionGroup>=32||b->state.collisionGroup>=32) return 0;
	if(a->state.ignoreAssembly && a->state.ignoreAssembly==b->state.ignoreAssembly) return 0;
	if(!(a->state.category&b->state.mask)||!(b->state.category&a->state.mask)) return 0;
	if(o&&o->groupMasks && (!(o->groupMasks[a->state.collisionGroup]&(UINT32_C(1)<<b->state.collisionGroup)) ||
		!(o->groupMasks[b->state.collisionGroup]&(UINT32_C(1)<<a->state.collisionGroup)))) return 0;
	return 1;
}
int YMPME_Body_GetAABB(const pme_body* b,pme_aabb* bounds)
{
	pme_vec3 center,half;
	if(!b||!b->active||!bounds) return 0;
	center=b->state.position;
	if(b->state.shape==PME_SPHERE) half=YMPME_Vec3(b->state.radius,b->state.radius,b->state.radius);
	else if(b->state.shape==PME_BOX) half=b->state.halfExtents;
	else
	{
		const pme_convex_mesh* m=b->state.mesh;
		pme_rotation r=b->state.rotation;
		pme_vec3 h,local;
		if(!m) return 0;
		h=YMPME_VecScale(YMPME_VecSub(m->bounds.max,m->bounds.min),0.5f);
		local=YMPME_VecScale(YMPME_VecAdd(m->bounds.max,m->bounds.min),0.5f);
		center=YMPME_VecAdd(center,YMPME_Rotation_Apply(r,local));
		half=YMPME_Vec3(fabsf(r.axis[0].x)*h.x+fabsf(r.axis[1].x)*h.y+fabsf(r.axis[2].x)*h.z,
			fabsf(r.axis[0].y)*h.x+fabsf(r.axis[1].y)*h.y+fabsf(r.axis[2].y)*h.z,
			fabsf(r.axis[0].z)*h.x+fabsf(r.axis[1].z)*h.y+fabsf(r.axis[2].z)*h.z);
	}
	bounds->min=YMPME_VecSub(center,half); bounds->max=YMPME_VecAdd(center,half);
	return 1;
}
int YMPME_CollideWithOptions(const pme_body* a,const pme_body* b,const pme_collision_options* o,pme_contact* contact)
{
	pme_contact c;
	pme_aabb aa,bb;
	int hit,mesh;
	pme_collision_stats* stats=o?o->stats:NULL;
	if(!YMPME_ShouldCollide(a,b,o)) { if(stats) ++stats->filteredPairs; return 0; }
	if(stats) ++stats->aabbTests;
	if(!YMPME_Body_GetAABB(a,&aa)||!YMPME_Body_GetAABB(b,&bb)) return 0;
	if(aa.max.x<bb.min.x||bb.max.x<aa.min.x||aa.max.y<bb.min.y||bb.max.y<aa.min.y||aa.max.z<bb.min.z||bb.max.z<aa.min.z)
	{
		if(stats) ++stats->aabbRejected;
		return 0;
	}
	mesh=a->state.shape==PME_CONVEX_MESH||b->state.shape==PME_CONVEX_MESH;
	if(mesh && o && !o->meshPrecision)
	{
		pme_body_desc ad=a->state,bd=b->state;
		ad.position=YMPME_VecScale(YMPME_VecAdd(aa.min,aa.max),0.5f);
		ad.halfExtents=YMPME_VecScale(YMPME_VecSub(aa.max,aa.min),0.5f);
		bd.position=YMPME_VecScale(YMPME_VecAdd(bb.min,bb.max),0.5f);
		bd.halfExtents=YMPME_VecScale(YMPME_VecSub(bb.max,bb.min),0.5f);
		hit=boxBox(&ad,&bd,&c);
	}
	else
	{
		if(stats) ++stats->narrowphaseTests;
		if(mesh) { if(stats) ++stats->meshTests; hit=YMPME_MeshSAT(a,b,&c); }
		else if(a->state.shape==PME_SPHERE&&b->state.shape==PME_SPHERE) hit=sphereSphere(&a->state,&b->state,&c);
		else if(a->state.shape==PME_BOX&&b->state.shape==PME_BOX) hit=boxBox(&a->state,&b->state,&c);
		else if(a->state.shape==PME_SPHERE) hit=sphereBox(&a->state,&b->state,&c);
		else { hit=sphereBox(&b->state,&a->state,&c); if(hit) c.normal=YMPME_VecScale(c.normal,-1); }
	}
	if(hit&&contact) *contact=c;
	return hit;
}
int YMPME_Collide(const pme_body* a,const pme_body* b,pme_contact* contact)
{
	return YMPME_CollideWithOptions(a,b,NULL,contact);
}
