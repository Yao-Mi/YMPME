#include "YMPME_Manifold.h"
#include "../CORE/YMPME_MeshSAT.h"
#include <float.h>
#include <string.h>
#define V YMPME_Vec3
#define add YMPME_VecAdd
#define sub YMPME_VecSub
#define mul YMPME_VecScale
#define dot YMPME_VecDot
#define cross YMPME_VecCross
static float norm(pme_vec3 p) { return sqrtf(dot(p, p)); }
static const uint32_t boxIndices[36] = {0, 4, 6, 0, 6, 2, 1, 3, 7, 1, 7, 5, 0, 1, 5, 0, 5, 4,
										2, 6, 7, 2, 7, 3, 0, 2, 3, 0, 3, 1, 4, 5, 7, 4, 7, 6};
typedef struct poly_
{
	pme_body proxy;
	pme_convex_mesh box;
	pme_vec3 vertices[8], axes[3], center;
} poly;
static void makePoly(poly *p, const pme_collider *c, const pme_collision_pose *b)
{
	memset(p, 0, sizeof(*p));
	p->proxy.state.position = b->position;
	p->proxy.state.rotation = YMPME_Quaternion_Rotation(b->orientation);
	p->proxy.state.shape = PME_CONVEX_MESH;
	if (c->type == PME_COLLIDER_BOX)
	{
		for (int i = 0; i < 8; ++i)
			p->vertices[i] = V((i & 1 ? 1 : -1) * c->halfSize.x, (i & 2 ? 1 : -1) * c->halfSize.y,
							   (i & 4 ? 1 : -1) * c->halfSize.z);
		p->axes[0] = V(1, 0, 0);
		p->axes[1] = V(0, 1, 0);
		p->axes[2] = V(0, 0, 1);
		p->box = (pme_convex_mesh){p->vertices, boxIndices, 8,
								   12,			p->axes,	p->axes,
								   3,			3,			{mul(c->halfSize, -1), c->halfSize}, NULL};
		p->proxy.state.mesh = &p->box;
	}
	else
		p->proxy.state.mesh = c->mesh;
	const pme_convex_mesh *m = p->proxy.state.mesh;
	pme_vec3 center = V(0, 0, 0);
	for (size_t i = 0; i < m->vertexCount; ++i)
		center = add(center, mul(m->vertices[i], 1.f / m->vertexCount));
	p->center = add(b->position, YMPME_Rotation_Apply(p->proxy.state.rotation, center));
}
static pme_vec3 vertex(const poly *p, size_t i)
{
	return add(p->proxy.state.position,
			   YMPME_Rotation_Apply(p->proxy.state.rotation, p->proxy.state.mesh->vertices[i]));
}
static pme_vec3 triangle(const poly *p, size_t i, pme_vec3 v[3])
{
	for (int k = 0; k < 3; ++k)
		v[k] = vertex(p, YMPME_ConvexMesh_Index(p->proxy.state.mesh,3 * i + k));
	pme_vec3 n = cross(sub(v[1], v[0]), sub(v[2], v[0]));
	n = mul(n, 1 / norm(n));
	if (dot(n, sub(p->center, v[0])) > 0)
		n = mul(n, -1);
	return n;
}
static void reduce(pme_manifold_point *p, size_t *count)
{
	if (*count <= 4)
		return;
	size_t pick[4] = {0, 0, 0, 0};
	float best;
	for (size_t i = 1; i < *count; ++i)
		if (p[i].depth > p[pick[0]].depth)
			pick[0] = i;
	best = -1;
	for (size_t i = 0; i < *count; ++i)
	{
		pme_vec3 d = sub(p[i].pointA, p[pick[0]].pointA);
		float score = dot(d, d);
		if (i != pick[0] && score > best)
		{
			pick[1] = i;
			best = score;
		}
	}
	best = -1;
	for (size_t i = 0; i < *count; ++i)
	{
		float score = norm(
			cross(sub(p[pick[1]].pointA, p[pick[0]].pointA), sub(p[i].pointA, p[pick[0]].pointA)));
		if (i != pick[0] && i != pick[1] && score > best)
		{
			pick[2] = i;
			best = score;
		}
	}
	best = -1;
	for (size_t i = 0; i < *count; ++i)
	{
		if (i == pick[0] || i == pick[1] || i == pick[2])
			continue;
		float score = 0;
		for (int k = 0; k < 3; ++k)
			score += norm(cross(sub(p[pick[k]].pointA, p[i].pointA),
								sub(p[pick[(k + 1) % 3]].pointA, p[i].pointA)));
		if (score > best)
		{
			pick[3] = i;
			best = score;
		}
	}
	pme_manifold_point tmp[4];
	for (int i = 0; i < 4; ++i)
		tmp[i] = p[pick[i]];
	memcpy(p, tmp, sizeof(tmp));
	*count = 4;
}
static void candidate(pme_manifold_point p[8], size_t *count, pme_manifold_point c, float eps)
{
	for (size_t i = 0; i < *count; ++i)
		if (norm(sub(p[i].pointA, c.pointA)) < eps && norm(sub(p[i].pointB, c.pointB)) < eps)
			return;
	if (*count == 8)
		reduce(p, count);
	p[(*count)++] = c;
}
/* Intersection of an incident triangle with the reference triangle's extruded
 * side planes. Triangle triangulation seams are deduplicated before reduction. */
static size_t clip(const pme_vec3 ref[3], pme_vec3 n, const pme_vec3 incident[3], pme_vec3 out[8],
				   float eps)
{
	pme_vec3 buffer[8];
	memcpy(out, incident, 3 * sizeof(*out));
	size_t count = 3;
	for (int edge = 0; edge < 3 && count; ++edge)
	{
		pme_vec3 side = cross(n, sub(ref[(edge + 1) % 3], ref[edge]));
		side = mul(side, 1 / norm(side));
		if (dot(side, sub(ref[(edge + 2) % 3], ref[edge])) < 0)
			side = mul(side, -1);
		size_t next = 0;
		pme_vec3 prev = out[count - 1];
		float dp = dot(side, sub(prev, ref[edge]));
		for (size_t i = 0; i < count; ++i)
		{
			pme_vec3 cur = out[i];
			float dc = dot(side, sub(cur, ref[edge]));
			if ((dp >= -eps) != (dc >= -eps))
			{
				float t = YMPME_Clamp(dp / (dp - dc), 0, 1);
				buffer[next++] = add(prev, mul(sub(cur, prev), t));
			}
			if (dc >= -eps)
				buffer[next++] = cur;
			prev = cur;
			dp = dc;
		}
		count = next;
		memcpy(out, buffer, count * sizeof(*out));
	}
	return count;
}
static void segments(pme_vec3 p, pme_vec3 q, pme_vec3 r, pme_vec3 s, pme_vec3 *a, pme_vec3 *b)
{
	pme_vec3 d1 = sub(q, p), d2 = sub(s, r), v = sub(p, r);
	float aa = dot(d1, d1), ee = dot(d2, d2), ff = dot(d2, v), u = 0, t = 0;
	if (aa < 1e-16f)
		t = ee > 1e-16f ? YMPME_Clamp(ff / ee, 0, 1) : 0;
	else
	{
		float cc = dot(d1, v);
		if (ee < 1e-16f)
			u = YMPME_Clamp(-cc / aa, 0, 1);
		else
		{
			float bb = dot(d1, d2), den = aa * ee - bb * bb;
			if (den > 1e-16f)
				u = YMPME_Clamp((bb * ff - cc * ee) / den, 0, 1);
			t = (bb * u + ff) / ee;
			if (t < 0)
			{
				t = 0;
				u = YMPME_Clamp(-cc / aa, 0, 1);
			}
			else if (t > 1)
			{
				t = 1;
				u = YMPME_Clamp((bb - cc) / aa, 0, 1);
			}
		}
	}
	*a = add(p, mul(d1, u));
	*b = add(r, mul(d2, t));
}
static float support(const poly *p, pme_vec3 n)
{
	float high = -FLT_MAX;
	for (size_t i = 0; i < p->proxy.state.mesh->vertexCount; ++i)
		high = fmaxf(high, dot(vertex(p, i), n));
	return high;
}
static int supportEdge(const poly *p, size_t i, pme_vec3 n, float high, float eps, pme_vec3 *a,
					   pme_vec3 *b)
{
	const pme_convex_mesh *m = p->proxy.state.mesh;
	if (i < m->triangleCount * 3)
	{
		*a = vertex(p, YMPME_ConvexMesh_Index(m,i));
		*b = vertex(p, YMPME_ConvexMesh_Index(m,i / 3 * 3 + (i + 1) % 3));
	}
	else
	{
		*a = *b = vertex(p, i - m->triangleCount * 3);
	}
	return high - dot(*a, n) <= eps && high - dot(*b, n) <= eps;
}
static size_t edgeContact(const poly *a, const poly *b, pme_contact sat, pme_manifold_point *out,
						  float eps)
{
	pme_vec3 an = sat.normal, bn = mul(an, -1), pa, pb, qa, qb;
	float ah = support(a, an), bh = support(b, bn), best = FLT_MAX;
	size_t na = a->proxy.state.mesh->triangleCount * 3 + a->proxy.state.mesh->vertexCount;
	size_t nb = b->proxy.state.mesh->triangleCount * 3 + b->proxy.state.mesh->vertexCount;
	int found = 0;
	for (size_t i = 0; i < na; ++i)
		if (supportEdge(a, i, an, ah, eps, &pa, &pb))
			for (size_t j = 0; j < nb; ++j)
				if (supportEdge(b, j, bn, bh, eps, &qa, &qb))
				{
					pme_vec3 x, y;
					segments(pa, pb, qa, qb, &x, &y);
					float d = dot(sub(x, y), sub(x, y));
					if (d < best)
					{
						*out = (pme_manifold_point){an, x, y, sat.penetration};
						best = d;
						found = 1;
					}
				}
	return (size_t)found;
}
size_t PME_ConvexManifold(const pme_collider *ca, const pme_collision_pose *ba, const pme_collider *cb,
						  const pme_collision_pose *bb, pme_manifold_point out[4])
{
	poly a, b;
	makePoly(&b, cb, bb);
	pme_manifold_point points[8];
	size_t count = 0;
	float eps =
		1e-6f + norm(sub(b.proxy.state.mesh->bounds.max, b.proxy.state.mesh->bounds.min)) * 1e-5f;
	if (ca->type == PME_COLLIDER_PLANE)
	{
		pme_vec3 n = YMPME_Quaternion_Rotation(ba->orientation).axis[1];
		for (size_t i = 0; i < b.proxy.state.mesh->vertexCount; ++i)
		{
			pme_vec3 p = vertex(&b, i);
			float d = dot(n, sub(p, ba->position));
			if (d <= 0)
				candidate(points, &count, (pme_manifold_point){n, sub(p, mul(n, d)), p, -d}, eps);
		}
	}
	else if (ca->type == PME_COLLIDER_SPHERE)
	{
		pme_vec3 closest = V(0, 0, 0), faceNormal = V(0, 1, 0);
		float best = FLT_MAX;
		int inside = 1;
		for (size_t i = 0; i < b.proxy.state.mesh->triangleCount; ++i)
		{
			pme_vec3 v[3], n = triangle(&b, i, v),
						   q = YMPME_Triangle_ClosestPoint(ba->position, v[0], v[1], v[2]);
			float d = dot(sub(q, ba->position), sub(q, ba->position));
			if (dot(n, sub(ba->position, v[0])) > eps)
				inside = 0;
			if (d < best)
			{
				best = d;
				closest = q;
				faceNormal = n;
			}
		}
		float distance = sqrtf(best), depth = ca->radius + (inside ? distance : -distance);
		if (depth >= 0)
		{
			pme_vec3 n = distance > eps
							 ? mul(sub(ba->position, closest), (inside ? -1 : 1) / distance)
							 : faceNormal;
			/* Original order is sphere A / mesh B; n above is mesh -> sphere. */
			out[0] = (pme_manifold_point){mul(n, -1), sub(ba->position, mul(n, ca->radius)),
										  closest, depth};
			return 1;
		}
		return 0;
	}
	else
	{
		makePoly(&a, ca, ba);
		pme_contact sat;
		if (!YMPME_MeshSAT(&a.proxy, &b.proxy, &sat))
			return 0;
		eps = fmaxf(
			eps, 1e-6f + norm(sub(a.proxy.state.mesh->bounds.max, a.proxy.state.mesh->bounds.min)) *
							 1e-5f);
		float bestA = -2, bestB = -2;
		for (size_t i = 0; i < a.proxy.state.mesh->triangleCount; ++i)
		{
			pme_vec3 v[3];
			bestA = fmaxf(bestA, dot(triangle(&a, i, v), sat.normal));
		}
		for (size_t i = 0; i < b.proxy.state.mesh->triangleCount; ++i)
		{
			pme_vec3 v[3];
			bestB = fmaxf(bestB, -dot(triangle(&b, i, v), sat.normal));
		}
		if (fmaxf(bestA, bestB) < .99999f)
			return edgeContact(&a, &b, sat, out, eps);
		int flip = bestB > bestA + 1e-6f;
		const poly *ref = flip ? &b : &a, *inc = flip ? &a : &b;
		pme_vec3 direction = mul(sat.normal, flip ? -1 : 1);
		float incident = -2;
		for (size_t i = 0; i < inc->proxy.state.mesh->triangleCount; ++i)
		{
			pme_vec3 v[3];
			incident = fmaxf(incident, -dot(triangle(inc, i, v), direction));
		}
		for (size_t i = 0; i < ref->proxy.state.mesh->triangleCount; ++i)
		{
			pme_vec3 rv[3], rn = triangle(ref, i, rv);
			if (dot(rn, direction) < .99999f)
				continue;
			for (size_t j = 0; j < inc->proxy.state.mesh->triangleCount; ++j)
			{
				pme_vec3 iv[3], in = triangle(inc, j, iv), clipped[8];
				if (-dot(in, direction) < incident - 1e-5f)
					continue;
				size_t n = clip(rv, rn, iv, clipped, eps);
				for (size_t k = 0; k < n; ++k)
				{
					float depth = dot(rn, sub(rv[0], clipped[k]));
					if (depth < -eps)
						continue;
					depth = fmaxf(0, depth);
					pme_vec3 q = add(clipped[k], mul(rn, depth));
					candidate(points, &count,
							  (pme_manifold_point){flip ? mul(rn, -1) : rn, flip ? clipped[k] : q,
												   flip ? q : clipped[k], depth},
							  eps);
				}
			}
		}
		if (!count)
			return edgeContact(&a, &b, sat, out, eps);
	}
	reduce(points, &count);
	memcpy(out, points, count * sizeof(*out));
	return count;
}
