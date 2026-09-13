#include "YMPME_RopeGRE.h"
#include "YMGRE/CONFIG/YMGRE_Mem.h"
#include "YMGRE/OPOBJ/YMGRE_Creat.h"
#include "YMGRE/OPOBJ/YMGRE_Free.h"
#include <string.h>
pme_result YMPME_GRE_Rope_Create(pme_rope_gre *b, const pme_rope *r, float units, float radius,
								 GRErgb24 color)
{
	if (!b || !r || !r->nodes || !r->links || r->count < 2 || r->count > 4096 || !isfinite(units) ||
		units <= 0 || units > 1e4 || !isfinite(radius) || radius <= 0 || radius > 100)
		return PME_INVALID_ARGUMENT;
	pme_rope_gre out;
	memset(&out, 0, sizeof(out));
	out.rope = r;
	out.nodes = r->nodes;
	out.count = r->count;
	out.linkCount = r->linkCount;
	out.closed = r->closed;
	out.unitsPerMeter = units;
	out.radius = radius;
	out.alpha = -1;
	out.object = YMGRE_Creat_Object((int)(8 * r->linkCount), (int)(12 * r->linkCount), "rope", "");
	if (!out.object)
		return PME_CAPACITY_EXCEEDED;
	static const uint16 triangles[12][3] = {{0, 2, 1}, {0, 3, 2}, {4, 5, 6}, {4, 6, 7}, {0, 1, 5}, {0, 5, 4},
											{1, 2, 6}, {1, 6, 5}, {2, 3, 7}, {2, 7, 6}, {3, 0, 4}, {3, 4, 7}};
	for (size_t e = 0; e < r->linkCount; ++e)
		for (int f = 0; f < 12; ++f)
		{
			GRE_Polygon4d p = &out.object->polygonList[e * 12 + f];
			p->num = 3;
			p->planeColor = color;
			p->ishide = 0;
			p->index = GRE_PolyIndex_Malloc(3 * sizeof(*p->index));
			if (!p->index)
			{
				YMGRE_Free_Object(out.object);
				return PME_CAPACITY_EXCEEDED;
			}
			for (int j = 0; j < 3; ++j)
				p->index[j] = (uint16)(e * 8 + triangles[f][j]);
		}
	out.object->boundType = GRE_Bounding_Sphere_R;
	*b = out;
	return PME_OK;
}
static pme_vec3 sample(const pme_rope_node *n, float alpha)
{
	return YMPME_VecAdd(YMPME_VecScale(n->previous, 1 - alpha), YMPME_VecScale(n->position, alpha));
}
pme_result YMPME_GRE_Rope_Sync(pme_rope_gre *b, float alpha)
{
	if (!b || !b->object || !b->rope || !isfinite(alpha) || alpha < 0 || alpha > 1)
		return PME_INVALID_ARGUMENT;
	const pme_rope *r = b->rope;
	if (r->nodes != b->nodes || r->count != b->count || r->linkCount != b->linkCount ||
		r->closed != b->closed)
		return PME_INVALID_ARGUMENT;
	b->bytesWritten = 0;
	if (r->revision == b->revision && alpha == b->alpha)
		return PME_OK;
	GRE_Object4d o = b->object;
	pme_vec3 low = YMPME_Vec3(INFINITY, INFINITY, INFINITY),
			 high = YMPME_Vec3(-INFINITY, -INFINITY, -INFINITY);
	for (size_t e = 0; e < r->linkCount; ++e)
	{
		pme_vec3 a = sample(&r->nodes[e], alpha), z = sample(&r->nodes[(e + 1) % r->count], alpha),
				 axis = YMPME_VecSub(z, a);
		float len = sqrtf(YMPME_VecDot(axis, axis));
		axis = len > 1e-8f ? YMPME_VecScale(axis, 1 / len) : YMPME_Vec3(0, 1, 0);
		pme_vec3 ref = fabsf(axis.z) < .9f ? YMPME_Vec3(0, 0, 1) : YMPME_Vec3(0, 1, 0),
				 u = YMPME_VecCross(ref, axis);
		u = YMPME_VecScale(u, 1 / sqrtf(YMPME_VecDot(u, u)));
		pme_vec3 v = YMPME_VecCross(axis, u);
		for (int k = 0; k < 8; ++k)
		{
			int c = k % 4;
			float sx = (c == 1 || c == 2) ? 1 : -1, sy = c >= 2 ? 1 : -1;
			pme_vec3 p = YMPME_VecAdd(
				k < 4 ? a : z,
				YMPME_VecScale(YMPME_VecAdd(YMPME_VecScale(u, sx), YMPME_VecScale(v, sy)), b->radius));
			p = YMPME_VecScale(p, b->unitsPerMeter);
			o->pointList[e * 8 + k].pos = (gre_fvector4d){p.x, p.y, p.z, 1};
			low.x = fminf(low.x, p.x);
			low.y = fminf(low.y, p.y);
			low.z = fminf(low.z, p.z);
			high.x = fmaxf(high.x, p.x);
			high.y = fmaxf(high.y, p.y);
			high.z = fmaxf(high.z, p.z);
		}
		for (int f = 0; f < 12; ++f)
		{
			GRE_Polygon4d p = &o->polygonList[e * 12 + f];
			gre_fvector4d a0 = o->pointList[p->index[0]].pos, a1 = o->pointList[p->index[1]].pos,
						  a2 = o->pointList[p->index[2]].pos;
			pme_vec3 n = YMPME_VecCross(YMPME_Vec3(a1.x - a0.x, a1.y - a0.y, a1.z - a0.z),
										YMPME_Vec3(a2.x - a0.x, a2.y - a0.y, a2.z - a0.z));
			p->pN = (gre_fvector4d){n.x, n.y, n.z, 0};
		}
	}
	pme_vec3 center = YMPME_VecScale(YMPME_VecAdd(low, high), .5f),
			 half = YMPME_VecScale(YMPME_VecSub(high, low), .5f);
	o->WorldCoordinate = (gre_fvector4d){center.x, center.y, center.z, 1};
	o->BoundingSphereR = sqrtf(YMPME_VecDot(half, half));
	b->revision = r->revision;
	b->alpha = alpha;
	++b->updates;
	b->bytesWritten = (8 + 12) * r->linkCount * sizeof(gre_fvector4d) + sizeof(o->WorldCoordinate) +
					  sizeof(o->BoundingSphereR);
	return PME_OK;
}
void YMPME_GRE_Rope_Destroy(pme_rope_gre *b)
{
	if (!b)
		return;
	if (b->object)
		YMGRE_Free_Object(b->object);
	memset(b, 0, sizeof(*b));
}

pme_result YMPME_GRE_Rope_Rebind(pme_rope_gre *b, const pme_rope *r)
{
	if (!b || !b->object || !r || !r->nodes || r->count != b->count || r->linkCount != b->linkCount ||
		r->closed != b->closed)
		return PME_INVALID_ARGUMENT;
	b->rope = r;
	b->nodes = r->nodes;
	b->revision = 0;
	b->alpha = -1;
	b->bytesWritten = 0;
	return PME_OK;
}
