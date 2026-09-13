#include "YMPME_Gear.h"
#include <string.h>
#define VMAX 22
#define CMAX 32
typedef struct tooth_
{
	pme_gear_point v[VMAX];
	double minx, maxx, miny, maxy;
} tooth;
typedef struct contact_
{
	pme_gear_point p, n;
	double depth, impulse, ca, cb, mass;
} contact;
static double cross(pme_gear_point a, pme_gear_point b)
{
	return a.x * b.y - a.y * b.x;
}
static pme_gear_point sub(pme_gear_point a, pme_gear_point b)
{
	return (pme_gear_point){a.x - b.x, a.y - b.y};
}
static double dot(pme_gear_point a, pme_gear_point b)
{
	return a.x * b.x + a.y * b.y;
}
static void transform(tooth *out, const pme_gear_point *in, int z, double angle, double x)
{
	int t, j;
	double c = cos(angle), s = sin(angle);
	for (t = 0; t < z; ++t)
	{
		tooth *p = &out[t];
		p->minx = p->miny = 1e30;
		p->maxx = p->maxy = -1e30;
		for (j = 0; j < 22; ++j)
		{
			pme_gear_point a = in[t * 23 + j + 1], b = {a.x * c - a.y * s + x, a.x * s + a.y * c};
			p->v[j] = b;
			p->minx = fmin(p->minx, b.x);
			p->maxx = fmax(p->maxx, b.x);
			p->miny = fmin(p->miny, b.y);
			p->maxy = fmax(p->maxy, b.y);
		}
	}
}
static int piece(const tooth *t, int root, pme_gear_point *p)
{
	int i;
	if (root)
	{
		p[0] = t->v[0];
		p[1] = t->v[1];
		p[2] = t->v[20];
		p[3] = t->v[21];
		return 4;
	}
	for (i = 0; i < 20; ++i)
		p[i] = t->v[i + 1];
	return 20;
}
/* 凸齿面/齿根片的 SAT，轴来自真实离散齿形，绝不使用齿数比修正速度。 */
static int collide(const pme_gear_point *a, int na, const pme_gear_point *b, int nb, contact *c)
{
	int pass, i, j;
	double depth = 1e30;
	pme_gear_point normal = {0, 0};
	for (pass = 0; pass < 2; ++pass)
	{
		const pme_gear_point *p = pass ? b : a;
		int n = pass ? nb : na;
		for (i = 0; i < n; ++i)
		{
			pme_gear_point e = sub(p[(i + 1) % n], p[i]), axis = {e.y, -e.x};
			double len = hypot(axis.x, axis.y), amin = 1e30, amax = -1e30, bmin = 1e30, bmax = -1e30, d;
			if (len < 1e-12)
				continue;
			axis.x /= len;
			axis.y /= len;
			for (j = 0; j < na; ++j)
			{
				double q = dot(a[j], axis);
				amin = fmin(amin, q);
				amax = fmax(amax, q);
			}
			for (j = 0; j < nb; ++j)
			{
				double q = dot(b[j], axis);
				bmin = fmin(bmin, q);
				bmax = fmax(bmax, q);
			}
			if (amax <= bmin || bmax <= amin)
				return 0;
			d = amax - bmin;
			if (bmax - amin < d)
			{
				d = bmax - amin;
				axis.x = -axis.x;
				axis.y = -axis.y;
			}
			if (d < depth)
			{
				depth = d;
				normal = axis;
			}
		}
	}
	/* 裁剪交叠多边形，接触点位于实际齿面交叠区域。 */
	{
		pme_gear_point polygon[64], next[64], center = {0, 0};
		int count = na;
		memcpy(polygon, a, (size_t)na * sizeof(*a));
		for (i = 0; i < nb && count; ++i)
		{
			int size = 0;
			pme_gear_point start = b[i], edge = sub(b[(i + 1) % nb], start);
			for (j = 0; j < count; ++j)
			{
				pme_gear_point u = polygon[j], v = polygon[(j + 1) % count];
				double du = cross(edge, sub(u, start)), dv = cross(edge, sub(v, start));
				if (du >= 0 && size < 64)
					next[size++] = u;
				if ((du >= 0) != (dv >= 0))
				{
					double t = du / (du - dv);
					if (size < 64)
						next[size++] = (pme_gear_point){u.x + (v.x - u.x) * t, u.y + (v.y - u.y) * t};
				}
			}
			count = size;
			memcpy(polygon, next, (size_t)count * sizeof(*next));
		}
		if (!count)
			return 0;
		for (i = 0; i < count; ++i)
		{
			center.x += polygon[i].x;
			center.y += polygon[i].y;
		}
		c->p = (pme_gear_point){center.x / count, center.y / count};
		c->n = normal;
		c->depth = depth;
		c->impulse = 0;
	}
	return 1;
}
static int detect(pme_gear_pair *p, contact *contacts)
{
	tooth a[20], b[30];
	int i, j, r, s, n = 0;
	transform(a, p->profile[0], 20, p->angle, 0);
	transform(b, p->profile[1], 30, p->angleB, p->distance);
	for (i = 0; i < 20; ++i)
		for (j = 0; j < 30; ++j)
		{
			contact best;
			int found = 0;
			best.depth = 0;
			if (a[i].maxx <= b[j].minx || b[j].maxx <= a[i].minx || a[i].maxy <= b[j].miny ||
				b[j].maxy <= a[i].miny)
				continue;
			++p->candidates;
			for (r = 0; r < 2; ++r)
				for (s = 0; s < 2; ++s)
				{
					pme_gear_point A[22], B[22];
					contact c;
					int na = piece(&a[i], r, A), nb = piece(&b[j], s, B);
					if (collide(A, na, B, nb, &c) && c.depth > best.depth)
					{
						best = c;
						found = 1;
					}
				}
			if (found && n < CMAX)
			{
				best.ca = cross(best.p, best.n);
				best.cb = cross(sub(best.p, (pme_gear_point){p->distance, 0}), best.n);
				best.mass = best.ca * best.ca / .8 + best.cb * best.cb / 2.4;
				if (best.mass > 1e-9)
					contacts[n++] = best;
			}
		}
	return n;
}
int YMPME_Gear_CollisionStep(pme_gear_pair *p, double dt)
{
	double h;
	int steps, step, i, k;
	if (!p || !isfinite(dt) || dt <= 0 || dt > .05)
		return 0;
	steps = (int)ceil(dt * 1200);
	h = dt / steps;
	if (!p->profileReady)
	{
		if (!YMPME_Gear_Profile(20, .12, p->profile[0], PME_GEAR_PROFILE_MAX) ||
			!YMPME_Gear_Profile(30, .12, p->profile[1], PME_GEAR_PROFILE_MAX))
			return 0;
		p->profileReady = 1;
	}
	p->contacts = 0;
	p->contactTorque = 0;
	p->maxPenetration = 0;
	p->candidates = 0;
	for (step = 0; step < steps; ++step)
	{
		contact c[CMAX];
		int count;
		p->torque = p->motor ? fmax(-8, fmin(8, 10 * (p->target - p->omega))) : 0;
		p->omega += (p->torque - .06 * p->omega) / .8 * h;
		/* 输出轴有轻微制动负载，让空程和换齿面过程容易观察。 */
		p->omegaB *= exp(-.06 / 2.4 * h);
		p->omegaB = copysign(fmax(0, fabs(p->omegaB) - p->loadTorque / 2.4 * h), p->omegaB);
		p->angle += p->omega * h;
		p->angleB += p->omegaB * h;
		count = detect(p, c);
		for (k = 0; k < 8; ++k)
			for (i = 0; i < count; ++i)
			{
				double speed = c[i].cb * p->omegaB - c[i].ca * p->omega;
				double increment = -speed / c[i].mass, old = c[i].impulse;
				c[i].impulse = fmax(0, old + increment);
				increment = c[i].impulse - old;
				p->omega -= c[i].ca * increment / .8;
				p->omegaB += c[i].cb * increment / 2.4;
			}
		for (i = 0; i < count; ++i)
		{
			double correction = fmin(.005, .65 * fmax(0, c[i].depth - .000002)) / c[i].mass;
			p->angle -= c[i].ca * correction / .8;
			p->angleB += c[i].cb * correction / 2.4;
			p->contactTorque += c[i].cb * c[i].impulse / dt;
			p->maxPenetration = fmax(p->maxPenetration, c[i].depth);
		}
		/* 保留这次外部物理步内的最后一批接触点，供 GRE 高亮。 */
		if (count)
		{
			p->contacts = count;
			p->pointCount = count < 8 ? count : 8;
			for (i = 0; i < p->pointCount; ++i)
				p->contactPoint[i] = c[i].p;
		}
		p->angle = remainder(p->angle, 2 * PME_GEAR_PI);
		p->angleB = remainder(p->angleB, 2 * PME_GEAR_PI);
	}
	if (!p->contacts)
		p->pointCount = 0;
	return isfinite(p->omega) && isfinite(p->omegaB) && isfinite(p->angle) && isfinite(p->angleB);
}
