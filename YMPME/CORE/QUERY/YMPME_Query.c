#include "YMPME_Query.h"
#include <float.h>
#define V YMPME_Vec3
#define ADD YMPME_VecAdd
#define SUB YMPME_VecSub
#define SCALE YMPME_VecScale
#define DOT YMPME_VecDot
#define CROSS YMPME_VecCross
static float component(pme_vec3 v, int i) { return i == 0 ? v.x : i == 1 ? v.y : v.z; }
static pme_vec3 axis(int i, float sign)
{
	return V(i == 0 ? sign : 0, i == 1 ? sign : 0, i == 2 ? sign : 0);
}
/* Slab intersection also preserves the outward normals for rays starting inside. */
static int slab(const pme_ray *r, pme_aabb box, float *distance, pme_vec3 *normal)
{
	double enter = -DBL_MAX, leave = DBL_MAX;
	pme_vec3 en = V(0, 0, 0), ex = en;
	int i;
	for (i = 0; i < 3; ++i)
	{
		double o = component(r->origin, i), d = component(r->direction, i),
			   lo = component(box.min, i), hi = component(box.max, i);
		double a, b;
		pme_vec3 na = axis(i, -1), nb = axis(i, 1);
		if (d == 0)
		{
			if (o < lo || o > hi)
				return 0;
			continue;
		}
		a = (lo - o) / d;
		b = (hi - o) / d;
		if (a > b)
		{
			double t = a;
			a = b;
			b = t;
			na = axis(i, 1);
			nb = axis(i, -1);
		}
		if (a > enter)
		{
			enter = a;
			en = na;
		}
		if (b < leave)
		{
			leave = b;
			ex = nb;
		}
		if (enter > leave)
			return 0;
	}
	if (leave < 0 || enter > r->maxDistance)
		return 0;
	if (enter >= 0)
	{
		*distance = (float)enter;
		*normal = en;
	}
	else
	{
		*distance = (float)leave;
		*normal = ex;
	}
	return *distance <= r->maxDistance;
}
static int sphere(const pme_ray *ray, const pme_body *b, float *t, pme_vec3 *normal)
{
	pme_vec3 v = SUB(ray->origin, b->state.position);
	double x = v.x, y = v.y, z = v.z,
		   d = x * ray->direction.x + y * ray->direction.y + z * ray->direction.z;
	double a = DOT(ray->direction, ray->direction),
		   c = x * x + y * y + z * z - (double)b->state.radius * b->state.radius;
	double disc = d * d - a * c, hit;
	if (disc < 0)
		return 0;
	hit = (-d - sqrt(disc)) / a;
	if (hit < 0)
		hit = (-d + sqrt(disc)) / a;
	if (hit < 0 || hit > ray->maxDistance)
		return 0;
	*t = (float)hit;
	*normal = SCALE(ADD(v, SCALE(ray->direction, *t)), 1 / b->state.radius);
	return 1;
}
static pme_vec3 local(pme_rotation r, pme_vec3 v)
{
	return V(DOT(r.axis[0], v), DOT(r.axis[1], v), DOT(r.axis[2], v));
}
static int mesh(const pme_ray *ray, const pme_body *b, float *distance, pme_vec3 *normal,
				size_t *primitive)
{
	const pme_convex_mesh *m = b->state.mesh;
	pme_vec3 o = local(b->state.rotation, SUB(ray->origin, b->state.position)),
			 d = local(b->state.rotation, ray->direction);
	pme_vec3 center;
	size_t i;
	int found = 0;
	center = V(0, 0, 0);
	for (i = 0; i < m->vertexCount; ++i)
		center = ADD(center, SCALE(m->vertices[i], 1.f / (float)m->vertexCount));
	for (i = 0; i < m->triangleCount; ++i)
	{
		pme_vec3 a = m->vertices[YMPME_ConvexMesh_Index(m,i * 3)],
				 e1 = SUB(m->vertices[YMPME_ConvexMesh_Index(m,i * 3 + 1)], a),
				 e2 = SUB(m->vertices[YMPME_ConvexMesh_Index(m,i * 3 + 2)], a);
		pme_vec3 p = CROSS(d, e2), v = SUB(o, a), q;
		double det = DOT(e1, p), u, w, t;
		double tolerance = 1e-8 * sqrt((double)DOT(e1, e1) * DOT(e2, e2));
		if (fabs(det) <= tolerance)
			continue;
		u = DOT(v, p) / det;
		if (u < -1e-7 || u > 1 + 1e-7)
			continue;
		q = CROSS(v, e1);
		w = DOT(d, q) / det;
		if (w < -1e-7 || u + w > 1 + 1e-7)
			continue;
		t = DOT(e2, q) / det;
		if (t < 0 || t > *distance)
			continue;
		*distance = (float)t;
		*normal = CROSS(e1, e2);
		if (DOT(*normal, SUB(center, a)) > 0)
			*normal = SCALE(*normal, -1);
		*normal = YMPME_Rotation_Apply(b->state.rotation,
									   SCALE(*normal, 1 / sqrtf(DOT(*normal, *normal))));
		*primitive = i;
		found = 1;
	}
	return found;
}
pme_query_filter YMPME_Query_DefaultFilter(void)
{
	return (pme_query_filter){UINT32_MAX, UINT32_MAX, 0, 0, SIZE_MAX, 0};
}
pme_result YMPME_Query_WorldRaycast(void *context, const pme_ray *r, pme_ray_hit *out)
{
	const pme_world_query *q = context;
	pme_ray_hit result = {0, 0, {0, 0, 0}, {0, 0, 0}, SIZE_MAX, SIZE_MAX};
	size_t i;
	if (!q || !q->world || !out || !r || (q->world->capacity && !q->world->bodies) ||
		q->filter.collisionGroup >= 32 || !YMPME_VecFinite(r->origin) ||
		!YMPME_VecFinite(r->direction) || fabsf(DOT(r->direction, r->direction) - 1) > 1e-4f ||
		!isfinite(r->maxDistance) || r->maxDistance <= 0)
		return PME_INVALID_ARGUMENT;
	result.distance = r->maxDistance;
	for (i = 0; i < q->world->capacity; ++i)
	{
		const pme_body *b = &q->world->bodies[i];
		pme_ray clipped = *r;
		pme_aabb bounds;
		pme_vec3 normal;
		float t;
		size_t primitive = SIZE_MAX;
		int hit;
		if (!b->active || i == q->filter.ignoreBody ||
			(!q->filter.includeSensors && b->state.sensor) ||
			!(b->state.category & q->filter.mask) || !(q->filter.category & b->state.mask) ||
			(q->filter.ignoreAssembly && q->filter.ignoreAssembly == b->state.ignoreAssembly))
			continue;
		if (b->state.collisionGroup >= 32)
			return PME_INVALID_ARGUMENT;
		if (!(q->world->groupMasks[q->filter.collisionGroup] &
			  (UINT32_C(1) << b->state.collisionGroup)))
			continue;
		clipped.maxDistance = result.distance;
		if (!YMPME_Body_GetAABB(b, &bounds))
			return PME_INVALID_ARGUMENT;
		/* The broad phase must retain an origin inside an AABB even when its exit is beyond range.
		 */
		if (!(r->origin.x >= bounds.min.x && r->origin.x <= bounds.max.x &&
			  r->origin.y >= bounds.min.y && r->origin.y <= bounds.max.y &&
			  r->origin.z >= bounds.min.z && r->origin.z <= bounds.max.z) &&
			!slab(&clipped, bounds, &t, &normal))
			continue;
		if (b->state.shape == PME_SPHERE)
			hit = sphere(&clipped, b, &t, &normal);
		else if (b->state.shape == PME_BOX)
			hit = slab(&clipped, bounds, &t, &normal);
		else if (b->state.shape == PME_CONVEX_MESH)
		{
			t = result.distance;
			hit = mesh(&clipped, b, &t, &normal, &primitive);
		}
		else
			return PME_INVALID_ARGUMENT;
		if (hit && (!result.hit || t < result.distance))
		{
			result.hit = 1;
			result.distance = t;
			result.point = ADD(r->origin, SCALE(r->direction, t));
			result.normal = normal;
			result.body = i;
			result.primitive = primitive;
		}
	}
	if (!result.hit)
		result.distance = 0;
	*out = result;
	return PME_OK;
}
