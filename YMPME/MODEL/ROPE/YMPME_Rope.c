#include "YMPME_Rope.h"
#include <string.h>
static int validPoint(pme_vec3 p)
{
	return YMPME_VecFinite(p) && fabsf(p.x) <= 1e5f && fabsf(p.y) <= 1e5f && fabsf(p.z) <= 1e5f;
}
static int validSettings(pme_vec3 gravity, float damping, float compliance, float maxSubstep,
						 unsigned iterations)
{
	return validPoint(gravity) && isfinite(damping) && damping >= 0 && damping <= 100 &&
		   isfinite(compliance) && compliance >= 0 && compliance <= 1 && isfinite(maxSubstep) &&
		   maxSubstep >= 1e-5f && maxSubstep <= .01f && iterations > 0 && iterations <= 256;
}
static int validMass(float m)
{
	return isfinite(m) && (m == 0 || (m >= 1e-6f && m <= 1e6f));
}
static double distance(pme_vec3 a, pme_vec3 b)
{
	double x = (double)a.x - b.x, y = (double)a.y - b.y, z = (double)a.z - b.z;
	return sqrt(x * x + y * y + z * z);
}
static int overlaps(const void *a, size_t an, const void *b, size_t bn)
{
	uintptr_t x = (uintptr_t)a, y = (uintptr_t)b;
	if (!a || !b)
		return 0;
	return x <= y ? y - x < an : x - y < bn;
}
static int buffers(PME_Rope r, pme_rope_node *n, size_t nc, pme_rope_link *l, size_t lc, size_t count,
				   size_t edges)
{
	if (!r || !n || !l)
		return PME_INVALID_ARGUMENT;
	if (nc < count || lc < edges)
		return PME_CAPACITY_EXCEEDED;
	if (overlaps(r, sizeof(*r), n, count * sizeof(*n)) || overlaps(r, sizeof(*r), l, edges * sizeof(*l)) ||
		overlaps(n, count * sizeof(*n), l, edges * sizeof(*l)))
		return PME_INVALID_ARGUMENT;
	return PME_OK;
}
pme_rope_desc YMPME_Rope_DefaultDesc(void)
{
	pme_rope_desc d;
	memset(&d, 0, sizeof(d));
	d.gravity = YMPME_Vec3(0, -9.81f, 0);
	d.damping = .05f;
	d.maxSubstep = 1.0f / 600;
	d.iterations = 24;
	return d;
}
static void setup(PME_Rope r, const pme_rope_desc *d, pme_rope_node *n, pme_rope_link *l)
{
	pme_rope out;
	memset(&out, 0, sizeof(out));
	out.nodes = n;
	out.links = l;
	out.count = d->count;
	out.closed = d->closed;
	out.linkCount = d->closed ? d->count : d->count - 1;
	out.gravity = d->gravity;
	out.damping = d->damping;
	out.compliance = d->compliance;
	out.maxSubstep = d->maxSubstep;
	out.iterations = d->iterations;
	out.revision = 1;
	*r = out;
}
pme_result YMPME_Rope_Init(PME_Rope r, const pme_rope_desc *d, pme_rope_node *n, size_t nc, pme_rope_link *l,
						   size_t lc)
{
	if (!d || d->count < 2 || d->count > 4096 || (d->closed != 0 && d->closed != 1) ||
		(d->closed && d->count < 3) || !d->positions || !d->masses || !validPoint(d->gravity) ||
		!isfinite(d->damping) || d->damping < 0 || d->damping > 100 || !isfinite(d->compliance) ||
		d->compliance < 0 || d->compliance > 1 || !isfinite(d->maxSubstep) || d->maxSubstep < 1e-5f ||
		d->maxSubstep > .01f || !d->iterations || d->iterations > 256)
		return PME_INVALID_ARGUMENT;
	size_t edges = d->closed ? d->count : d->count - 1;
	int result = buffers(r, n, nc, l, lc, d->count, edges);
	if (result)
		return (pme_result)result;
	const void *in[] = {d, d->positions, d->masses, d->restLengths};
	size_t sizes[] = {sizeof(*d), d->count * sizeof(*d->positions), d->count * sizeof(*d->masses),
					  edges * sizeof(float)};
	for (size_t i = 0; i < 4; ++i)
		if (overlaps(in[i], sizes[i], r, sizeof(*r)) || overlaps(in[i], sizes[i], n, d->count * sizeof(*n)) ||
			overlaps(in[i], sizes[i], l, edges * sizeof(*l)))
			return PME_INVALID_ARGUMENT;
	for (size_t i = 0; i < d->count; ++i)
		if (!validPoint(d->positions[i]) || !validMass(d->masses[i]))
			return PME_INVALID_ARGUMENT;
	for (size_t i = 0; i < edges; ++i)
	{
		double len =
			d->restLengths ? d->restLengths[i] : distance(d->positions[i], d->positions[(i + 1) % d->count]);
		if (!isfinite(len) || len < 1e-5 || len > 1e4)
			return PME_INVALID_ARGUMENT;
	}
	for (size_t i = 0; i < d->count; ++i)
	{
		memset(&n[i], 0, sizeof(n[i]));
		n[i].position = n[i].previous = n[i].old = n[i].target = d->positions[i];
		n[i].inverseMass = d->masses[i] ? 1 / d->masses[i] : 0;
	}
	for (size_t i = 0; i < edges; ++i)
	{
		memset(&l[i], 0, sizeof(l[i]));
		l[i].restLength = d->restLengths ? d->restLengths[i]
										 : (float)distance(d->positions[i], d->positions[(i + 1) % d->count]);
	}
	setup(r, d, n, l);
	return PME_OK;
}
pme_result YMPME_Rope_InitSpan(PME_Rope r, pme_rope_node *n, size_t nc, pme_rope_link *l, size_t lc,
							   size_t count, pme_vec3 start, pme_vec3 end, float length, float mass,
							   int pinStart, int pinEnd)
{
	if (count < 2 || count > 4096 || !validPoint(start) || !validPoint(end) || !validMass(mass) ||
		mass == 0 || !isfinite(length) || length / (count - 1) < 1e-5 || length / (count - 1) > 1e4 ||
		length + 1e-6 < distance(start, end) || (pinStart != 0 && pinStart != 1) ||
		(pinEnd != 0 && pinEnd != 1))
		return PME_INVALID_ARGUMENT;
	int result = buffers(r, n, nc, l, lc, count, count - 1);
	if (result)
		return (pme_result)result;
	for (size_t i = 0; i < count; ++i)
	{
		float t = (float)i / (count - 1);
		memset(&n[i], 0, sizeof(n[i]));
		n[i].position = n[i].previous = n[i].old = n[i].target =
			YMPME_VecAdd(YMPME_VecScale(start, 1 - t), YMPME_VecScale(end, t));
		n[i].inverseMass = ((!i && pinStart) || (i == count - 1 && pinEnd)) ? 0 : 1 / mass;
	}
	for (size_t i = 0; i < count - 1; ++i)
	{
		memset(&l[i], 0, sizeof(l[i]));
		l[i].restLength = length / (count - 1);
	}
	pme_rope_desc d = YMPME_Rope_DefaultDesc();
	d.count = count;
	setup(r, &d, n, l);
	return PME_OK;
}
static void constrain(PME_Rope r, size_t e, double h)
{
	pme_rope_node *a = &r->nodes[e], *b = &r->nodes[(e + 1) % r->count];
	pme_rope_link *l = &r->links[e];
	double w = (double)a->inverseMass + b->inverseMass, len = distance(a->position, b->position);
	if (w == 0 || len < 1e-12)
		return;
	double alpha = r->compliance / (h * h), dl = (-(len - l->restLength) - alpha * l->lambda) / (w + alpha);
	/* 单向 XPBD 约束：lambda <= 0，只能产生张力，松弛时卸载。 */
	double next = fmin(0, l->lambda + dl);
	dl = next - l->lambda;
	l->lambda = next;
	pme_vec3 d = YMPME_VecScale(YMPME_VecSub(b->position, a->position), (float)(dl / len));
	a->position = YMPME_VecSub(a->position, YMPME_VecScale(d, a->inverseMass));
	b->position = YMPME_VecAdd(b->position, YMPME_VecScale(d, b->inverseMass));
}
pme_result YMPME_Rope_Step(PME_Rope r, float dt)
{
	if (!r || !r->nodes || !r->links || r->count < 2 || r->count > 4096 ||
		(r->closed != 0 && r->closed != 1) || (r->closed && r->count < 3) ||
		r->linkCount != (r->closed ? r->count : r->count - 1) ||
		!validSettings(r->gravity, r->damping, r->compliance, r->maxSubstep, r->iterations) ||
		!isfinite(dt) || dt < 1e-6f || dt > .05f)
		return PME_INVALID_ARGUMENT;
	unsigned steps = (unsigned)ceil((double)dt / r->maxSubstep);
	double h = (double)dt / steps;
	float damping = (float)exp(-r->damping * h);
	for (size_t i = 0; i < r->count; ++i)
		r->nodes[i].previous = r->nodes[i].position;
	for (unsigned s = 0; s < steps; ++s)
	{
		for (size_t i = 0; i < r->count; ++i)
		{
			pme_rope_node *n = &r->nodes[i];
			n->old = n->position;
			if (n->inverseMass)
			{
				n->velocity =
					YMPME_VecScale(YMPME_VecAdd(n->velocity, YMPME_VecScale(r->gravity, (float)h)), damping);
				n->position = YMPME_VecAdd(n->position, YMPME_VecScale(n->velocity, (float)h));
			}
			else
			{
				float t = (float)(s + 1) / steps;
				n->position = YMPME_VecAdd(YMPME_VecScale(n->previous, 1 - t), YMPME_VecScale(n->target, t));
			}
		}
		for (size_t i = 0; i < r->linkCount; ++i)
			r->links[i].lambda = 0;
		for (unsigned k = 0; k < r->iterations; ++k)
			for (size_t j = 0; j < r->linkCount; ++j)
				constrain(r, k % 2 ? r->linkCount - 1 - j : j, h);
		for (size_t i = 0; i < r->count; ++i)
			r->nodes[i].velocity =
				YMPME_VecScale(YMPME_VecSub(r->nodes[i].position, r->nodes[i].old), (float)(1 / h));
		/* 消除绷紧绳段的向外相对速度；有柔度时保留弹性响应。 */
		if (r->compliance == 0)
			for (size_t e = 0; e < r->linkCount; ++e)
			{
				pme_rope_node *a = &r->nodes[e], *b = &r->nodes[(e + 1) % r->count];
				double len = distance(a->position, b->position), w = (double)a->inverseMass + b->inverseMass;
				if (r->links[e].lambda < 0 && len > 1e-12 && w > 0)
				{
					pme_vec3 n = YMPME_VecScale(YMPME_VecSub(b->position, a->position), (float)(1 / len));
					double outward = YMPME_VecDot(YMPME_VecSub(b->velocity, a->velocity), n);
					if (outward > 0)
					{
						a->velocity = YMPME_VecAdd(a->velocity,
												   YMPME_VecScale(n, (float)(outward * a->inverseMass / w)));
						b->velocity = YMPME_VecSub(b->velocity,
												   YMPME_VecScale(n, (float)(outward * b->inverseMass / w)));
					}
				}
			}
		for (size_t i = 0; i < r->linkCount; ++i)
			r->links[i].tension = -r->links[i].lambda / (h * h);
	}
	r->time += dt;
	++r->revision;
	return PME_OK;
}
pme_result YMPME_Rope_SetAnchor(PME_Rope r, size_t i, pme_vec3 target)
{
	if (!r || i >= r->count || !validPoint(target) || r->nodes[i].inverseMass)
		return PME_INVALID_ARGUMENT;
	r->nodes[i].target = target;
	return PME_OK;
}
pme_result YMPME_Rope_SetMass(PME_Rope r, size_t i, float mass)
{
	if (!r || i >= r->count || !validMass(mass))
		return PME_INVALID_ARGUMENT;
	pme_rope_node *n = &r->nodes[i];
	if ((mass == 0) != (n->inverseMass == 0))
		n->velocity = YMPME_Vec3(0, 0, 0);
	n->inverseMass = mass ? 1 / mass : 0;
	n->target = n->position;
	++r->revision;
	return PME_OK;
}
pme_result YMPME_Rope_AddImpulse(PME_Rope r, size_t i, pme_vec3 impulse)
{
	if (!r || i >= r->count || !validPoint(impulse) || !r->nodes[i].inverseMass)
		return PME_INVALID_ARGUMENT;
	pme_vec3 v = YMPME_VecAdd(r->nodes[i].velocity, YMPME_VecScale(impulse, r->nodes[i].inverseMass));
	if (!validPoint(v))
		return PME_INVALID_ARGUMENT;
	r->nodes[i].velocity = v;
	++r->revision;
	return PME_OK;
}
pme_rope_view YMPME_Rope_GetView(const pme_rope *r)
{
	pme_rope_view v;
	memset(&v, 0, sizeof(v));
	if (r)
	{
		v.nodes = r->nodes;
		v.links = r->links;
		v.count = r->count;
		v.linkCount = r->linkCount;
		v.closed = r->closed;
		v.revision = r->revision;
	}
	return v;
}
