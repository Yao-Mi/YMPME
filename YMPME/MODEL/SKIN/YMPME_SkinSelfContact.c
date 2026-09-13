#include "YMPME_SkinSelfContact.h"
#include "../../CORE/YMPME_MathBase.h"
#include <string.h>
#include <float.h>
#define V YMPME_Vec3
static float dot(pme_vec3 a, pme_vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
static pme_vec3 add(pme_vec3 a, pme_vec3 b) { return V(a.x + b.x, a.y + b.y, a.z + b.z); }
static pme_vec3 sub(pme_vec3 a, pme_vec3 b) { return V(a.x - b.x, a.y - b.y, a.z - b.z); }
static pme_vec3 mul(pme_vec3 a, float t) { return V(a.x * t, a.y * t, a.z * t); }
/* Barycentric projection onto the plane, with edge candidates outside the triangle. */
static pme_vec3 closest(pme_vec3 p, const pme_vec3 *v)
{
	pme_vec3 ab = sub(v[1], v[0]), ac = sub(v[2], v[0]), ap = sub(p, v[0]);
	pme_vec3 n = YMPME_VecCross(ab, ac);
	double d = (double)n.x * n.x + (double)n.y * n.y + (double)n.z * n.z;
	pme_vec3 ca = YMPME_VecCross(ap, ac), cb = YMPME_VecCross(ab, ap);
	float u = (float)(((double)ca.x * n.x + (double)ca.y * n.y + (double)ca.z * n.z) / d);
	float w = (float)(((double)cb.x * n.x + (double)cb.y * n.y + (double)cb.z * n.z) / d);
	if (u >= 0 && w >= 0 && u + w <= 1)
		return add(v[0], add(mul(ab, u), mul(ac, w)));
	pme_vec3 best = v[0];
	float distance = INFINITY;
	for (int i = 0; i < 3; i++)
	{
		pme_vec3 e = sub(v[(i + 1) % 3], v[i]);
		pme_vec3 q = add(v[i], mul(e, YMPME_Clamp(dot(sub(p, v[i]), e) / dot(e, e), 0, 1)));
		float z = dot(sub(p, q), sub(p, q));
		if (z < distance)
		{
			distance = z;
			best = q;
		}
	}
	return best;
}
static int near(pme_vec3 p, const pme_skin_self_target *box)
{
	return p.x >= box->low.x && p.x <= box->high.x && p.y >= box->low.y && p.y <= box->high.y &&
		   p.z >= box->low.z && p.z <= box->high.z;
}
pme_skin_self_desc YMPME_SkinSelfContact_DefaultDesc(void)
{
	pme_skin_self_desc d = {0};
	d.material = YMPME_SkinContact_DefaultDesc();
	d.thickness = .008f;
	d.searchDistance = .06f;
	return d;
}
pme_result YMPME_SkinSelfContact_Solve(const pme_skin_self_desc *d, const pme_skin_face *f,
									   size_t n, pme_vec3 ref, pme_vec3 *forces, pme_vec3 *moments,
									   pme_skin_self_target *targets, size_t cap,
									   pme_skin_self_result *out)
{
	if (!d || !out || (n && (!f || !forces || !moments || !targets)) || !YMPME_VecFinite(ref) ||
		!isfinite(d->thickness) || d->thickness <= 0 || !isfinite(d->searchDistance) ||
		d->searchDistance < d->thickness || !isfinite(d->searchDistance * d->searchDistance) ||
		(d->sourceRegions & d->targetRegions))
		return PME_INVALID_ARGUMENT;
	if (cap < n)
		return PME_CAPACITY_EXCEEDED;
	const pme_skin_contact_desc *m = &d->material;
	if (!isfinite(m->stiffness) || m->stiffness < 0 || !isfinite(m->damping) || m->damping < 0 ||
		!isfinite(m->friction) || m->friction < 0 || !isfinite(m->tangentDamping) ||
		m->tangentDamping < 0 || !isfinite(m->maxPressure) || m->maxPressure <= 0)
		return PME_INVALID_ARGUMENT;
	if (n > SIZE_MAX / sizeof(*forces))
		return PME_INVALID_ARGUMENT;
	pme_vec3 low[32], high[32];
	uint32_t present = 0, pairs[32] = {0};
	for (int g = 0; g < 32; g++)
	{
		low[g] = V(INFINITY, INFINITY, INFINITY);
		high[g] = V(-INFINITY, -INFINITY, -INFINITY);
	}
	double forceBound = 0, extent = fmax(fabs(ref.x), fmax(fabs(ref.y), fabs(ref.z)));
	for (size_t i = 0; i < n; i++)
	{
		const pme_surface_patch *p = &f[i].surface;
		if (f[i].region >= 32 || !YMPME_VecFinite(p->position) || !YMPME_VecFinite(p->normal) ||
			!YMPME_VecFinite(p->velocity) || !isfinite(16 * dot(p->velocity, p->velocity)) ||
			!isfinite(16 * dot(p->position, p->position)) || !isfinite(p->area) || p->area <= 0 ||
			fabsf(dot(p->normal, p->normal) - 1) > 1e-4f)
			return PME_INVALID_ARGUMENT;
		unsigned g = f[i].region;
		present |= UINT32_C(1) << g;
		forceBound += (double)m->maxPressure * p->area * (1 + (double)m->friction);
		extent =
			fmax(extent, fmax(fabs(p->position.x), fmax(fabs(p->position.y), fabs(p->position.z))));
		for (int k = 0; k < 3; k++)
		{
			pme_vec3 v = f[i].vertex[k];
			if (!YMPME_VecFinite(v) || !isfinite(16 * dot(v, v)))
				return PME_INVALID_ARGUMENT;
			extent = fmax(extent, fmax(fabs(v.x), fmax(fabs(v.y), fabs(v.z))));
			low[g] = V(fminf(low[g].x, v.x), fminf(low[g].y, v.y), fminf(low[g].z, v.z));
			high[g] = V(fmaxf(high[g].x, v.x), fmaxf(high[g].y, v.y), fmaxf(high[g].z, v.z));
		}
		pme_vec3 c = YMPME_VecCross(sub(f[i].vertex[1], f[i].vertex[0]),
									sub(f[i].vertex[2], f[i].vertex[0]));
		if (!isfinite(dot(c, c)) || dot(c, c) < 1e-20f || !isfinite(m->maxPressure * p->area))
			return PME_INVALID_ARGUMENT;
	}
	if (!isfinite(forceBound) || forceBound > FLT_MAX / 16.0 ||
		forceBound * extent > FLT_MAX / 64.0)
		return PME_INVALID_ARGUMENT;
	/* Region AABBs reject separated parts before per-sample triangle AABBs and closest points. */
	for (unsigned a = 0; a < 32; a++)
		for (unsigned b = 0; b < 32; b++)
		{
			uint32_t aa = UINT32_C(1) << a, bb = UINT32_C(1) << b;
			if (!(present & aa) || !(present & bb) || !(d->sourceRegions & aa) ||
				!(d->targetRegions & bb) || !(d->allow[a] & bb) || !(d->allow[b] & aa))
				continue;
			float r = d->searchDistance;
			if (low[a].x <= high[b].x + r && high[a].x >= low[b].x - r &&
				low[a].y <= high[b].y + r && high[a].y >= low[b].y - r &&
				low[a].z <= high[b].z + r && high[a].z >= low[b].z - r)
				pairs[a] |= bb;
		}
	if (n)
	{
		memset(forces, 0, n * sizeof(*forces));
		memset(moments, 0, n * sizeof(*moments));
	}
	pme_skin_self_result r = {0};
	size_t targetCount = 0;
	for (size_t j = 0; j < n; j++)
		if ((UINT32_C(1) << f[j].region) & d->targetRegions)
		{
			const pme_vec3 *v = f[j].vertex;
			float r = d->searchDistance;
			targets[targetCount++] = (pme_skin_self_target){
				j,
				{fminf(v[0].x, fminf(v[1].x, v[2].x)) - r, fminf(v[0].y, fminf(v[1].y, v[2].y)) - r,
				 fminf(v[0].z, fminf(v[1].z, v[2].z)) - r},
				{fmaxf(v[0].x, fmaxf(v[1].x, v[2].x)) + r, fmaxf(v[0].y, fmaxf(v[1].y, v[2].y)) + r,
				 fmaxf(v[0].z, fmaxf(v[1].z, v[2].z)) + r}};
		}
	for (size_t i = 0; i < n; i++)
	{
		uint32_t a = UINT32_C(1) << f[i].region;
		if (!(a & d->sourceRegions) || !pairs[f[i].region])
			continue;
		for (int sample = 0; sample < 4; sample++)
		{
			pme_vec3 p = sample == 0 ? f[i].surface.position : f[i].vertex[sample - 1];
			float best = d->searchDistance * d->searchDistance;
			size_t jbest = n;
			pme_vec3 qbest = V(0, 0, 0);
			for (size_t at = 0; at < targetCount; at++)
			{
				size_t j = targets[at].face;
				uint32_t b = UINT32_C(1) << f[j].region;
				if (!(pairs[f[i].region] & b) ||
					dot(f[i].surface.normal, f[j].surface.normal) > -.1f || !near(p, &targets[at]))
					continue;
				++r.candidates;
				pme_vec3 q = closest(p, f[j].vertex), v = sub(p, q);
				float dist = dot(v, v);
				if (dist < best)
				{
					best = dist;
					jbest = j;
					qbest = q;
				}
			}
			if (jbest == n)
				continue;
			size_t j = jbest;
			pme_vec3 normal = f[j].surface.normal, delta = sub(p, qbest);
			float gap = dot(delta, normal), depth = d->thickness - gap;
			/* Reject points past triangle edges; shell acts locally, never attracts skin. */
			pme_vec3 lateral = sub(delta, mul(normal, gap));
			if (depth <= 0 || dot(lateral, lateral) > d->thickness * d->thickness)
				continue;
			r.maxDepth = fmaxf(r.maxDepth, depth);
			pme_vec3 relative = sub(f[i].surface.velocity, f[j].surface.velocity);
			float vn = dot(relative, normal), area = f[i].surface.area * .25f;
			double pressure = (double)m->stiffness * depth - (double)m->damping * vn;
			r.saturated |= pressure > m->maxPressure;
			float fn = (float)(fmax(0, fmin(m->maxPressure, pressure)) * area);
			if (fn <= 0)
				continue;
			pme_vec3 tangent = sub(relative, mul(normal, vn));
			float speed = sqrtf(dot(tangent, tangent));
			pme_vec3 force = mul(normal, fn);
			if (speed > 1e-9f)
				force =
					sub(force, mul(tangent, (float)fmin((double)m->friction * fn,
														(double)m->tangentDamping * area * speed) /
												speed));
			pme_vec3 c = mul(add(p, qbest), .5f);
			forces[i] = add(forces[i], force);
			forces[j] = sub(forces[j], force);
			moments[i] = add(moments[i], YMPME_VecCross(sub(c, f[i].surface.position), force));
			moments[j] = sub(moments[j], YMPME_VecCross(sub(c, f[j].surface.position), force));
			r.sourceWrench.force = add(r.sourceWrench.force, force);
			r.sourceWrench.torque = add(r.sourceWrench.torque, YMPME_VecCross(sub(c, ref), force));
			++r.contacts;
		}
	}
	r.targetWrench.force = mul(r.sourceWrench.force, -1);
	r.targetWrench.torque = mul(r.sourceWrench.torque, -1);
	*out = r;
	return PME_OK;
}
