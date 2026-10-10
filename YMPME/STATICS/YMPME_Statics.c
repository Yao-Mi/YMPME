#include "YMPME_Statics.h"
#include <float.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

size_t pme_statics_workspace_doubles(size_t node_count)
{
	size_t n, limit = SIZE_MAX / sizeof(double);
	if (!node_count || node_count > limit / 3)
		return 0;
	n = 3 * node_count;
	if (n > limit / 3)
		return 0;
	if (n > (limit - 3 * n) / 2 / n)
		return 0;
	return 2 * n * n + 3 * n;
}

/* Build local stiffness and the global->local displacement transformation. */
static pme_statics_status element_matrices(const pme_statics_model *m, const pme_statics_element *e,
										   double k[36], double t[36], size_t dof[6])
{
	double dx, dy, length, c, s, axial;
	size_t i;
	if (e->node_a >= m->node_count || e->node_b >= m->node_count || e->node_a == e->node_b ||
		e->kind < PME_STATICS_SPRING || e->kind > PME_STATICS_BEAM)
		return PME_STATICS_INVALID_ARGUMENT;
	dx = m->nodes[e->node_b].x - m->nodes[e->node_a].x;
	dy = m->nodes[e->node_b].y - m->nodes[e->node_a].y;
	length = hypot(dx, dy);
	if (!(length > 0) || !isfinite(length))
		return PME_STATICS_INVALID_ARGUMENT;
	c = dx / length;
	s = dy / length;
	memset(k, 0, 36 * sizeof(double));
	memset(t, 0, 36 * sizeof(double));
	for (i = 0; i < 2; ++i)
	{
		size_t b = i * 3;
		t[b * 6 + b] = c;
		t[b * 6 + b + 1] = s;
		t[(b + 1) * 6 + b] = -s;
		t[(b + 1) * 6 + b + 1] = c;
		t[(b + 2) * 6 + b + 2] = 1;
		dof[b] = 3 * (i ? e->node_b : e->node_a);
		dof[b + 1] = dof[b] + 1;
		dof[b + 2] = dof[b] + 2;
	}
	if (e->kind == PME_STATICS_SPRING)
	{
		if (!(e->stiffness > 0) || !isfinite(e->stiffness))
			return PME_STATICS_INVALID_ARGUMENT;
		axial = e->stiffness;
	}
	else
	{
		if (!(e->young_modulus > 0) || !isfinite(e->young_modulus) || !(e->area > 0) ||
			!isfinite(e->area))
			return PME_STATICS_INVALID_ARGUMENT;
		axial = e->young_modulus * (e->area / length);
	}
	if (!(axial > 0) || !isfinite(axial))
		return PME_STATICS_NUMERIC_FAILURE;
	k[0] = k[21] = axial;
	k[3] = k[18] = -axial;
	if (e->kind == PME_STATICS_BEAM)
	{
		double h, b, v;
		if (!(e->second_moment > 0) || !isfinite(e->second_moment))
			return PME_STATICS_INVALID_ARGUMENT;
		h = e->young_modulus * (e->second_moment / length);
		b = 6 * h / length;
		v = 12 * (h / length) / length;
		if (!(h > 0) || !(b > 0) || !(v > 0) || !isfinite(4 * h) || !isfinite(b) || !isfinite(v))
			return PME_STATICS_NUMERIC_FAILURE;
		k[7] = k[28] = v;
		k[10] = k[25] = -v;
		k[8] = k[13] = k[11] = k[31] = b;
		k[16] = k[26] = k[29] = k[34] = -b;
		k[14] = k[35] = 4 * h;
		k[17] = k[32] = 2 * h;
	}
	return PME_STATICS_OK;
}

static int fixed_dof(const pme_statics_model *m, size_t i, double diagonal)
{
	const pme_statics_node *node = &m->nodes[i / 3];
	return (node->fixed & (1u << (i % 3))) || (i % 3 == 2 && diagonal == 0 && node->load[2] == 0);
}

pme_statics_status pme_statics_solve(const pme_statics_model *m, pme_statics_node_result *nodes,
									 size_t node_capacity, pme_statics_element_result *elements,
									 size_t element_capacity, double *workspace,
									 size_t workspace_doubles)
{
	size_t needed, n, i, j, l, e;
	double *k, *a, *b, *u, *scale;
	if (!m || !m->nodes || !m->node_count || !nodes || !workspace ||
		(m->element_count && (!m->elements || !elements)))
		return PME_STATICS_INVALID_ARGUMENT;
	needed = pme_statics_workspace_doubles(m->node_count);
	if (!needed || workspace_doubles < needed || node_capacity < m->node_count ||
		element_capacity < m->element_count)
		return PME_STATICS_CAPACITY_EXCEEDED;
	n = m->node_count * 3;
	k = workspace;
	a = k + n * n;
	b = a + n * n;
	u = b + n;
	scale = u + n;
	for (i = 0; i < m->node_count; ++i)
	{
		const pme_statics_node *node = &m->nodes[i];
		if (!isfinite(node->x) || !isfinite(node->y) || (node->fixed & ~7u))
			return PME_STATICS_INVALID_ARGUMENT;
		for (j = 0; j < 3; ++j)
			if (!isfinite(node->load[j]) || !isfinite(node->prescribed[j]) ||
				(!(node->fixed & (1u << j)) && node->prescribed[j] != 0))
				return PME_STATICS_INVALID_ARGUMENT;
	}
	memset(k, 0, n * n * sizeof(double));
	for (e = 0; e < m->element_count; ++e)
	{
		double local[36], t[36], kt[36] = {0};
		size_t dof[6];
		pme_statics_status status = element_matrices(m, &m->elements[e], local, t, dof);
		if (status != PME_STATICS_OK)
			return status;
		for (i = 0; i < 6; ++i)
			for (j = 0; j < 6; ++j)
				for (l = 0; l < 6; ++l)
					kt[i * 6 + j] += local[i * 6 + l] * t[l * 6 + j];
		for (i = 0; i < 6; ++i)
			for (j = 0; j < 6; ++j)
				for (l = 0; l < 6; ++l)
					k[dof[i] * n + dof[j]] += t[l * 6 + i] * kt[l * 6 + j];
	}
	for (i = 0; i < n * n; ++i)
		if (!isfinite(k[i]))
			return PME_STATICS_NUMERIC_FAILURE;
	/* Eliminate prescribed values and equilibrate units before Cholesky. */
	for (i = 0; i < n; ++i)
	{
		int fixed = fixed_dof(m, i, k[i * n + i]);
		u[i] = fixed ? m->nodes[i / 3].prescribed[i % 3] : 0;
		if (!fixed && !(k[i * n + i] > 0))
			return PME_STATICS_SINGULAR;
		scale[i] = fixed ? 1 : 1 / sqrt(k[i * n + i]);
	}
	for (i = 0; i < n; ++i)
	{
		int fixed = fixed_dof(m, i, k[i * n + i]);
		b[i] = fixed ? u[i] : m->nodes[i / 3].load[i % 3];
		for (j = 0; j < n; ++j)
		{
			int other_fixed = fixed_dof(m, j, k[j * n + j]);
			if (!fixed && other_fixed)
				b[i] -= k[i * n + j] * u[j];
			a[i * n + j] =
				fixed || other_fixed ? (i == j ? 1 : 0) : (k[i * n + j] * scale[i]) * scale[j];
		}
		b[i] *= scale[i];
		if (!isfinite(b[i]))
			return PME_STATICS_NUMERIC_FAILURE;
	}
	for (i = 0; i < n; ++i)
	{
		for (j = 0; j <= i; ++j)
		{
			double value = a[i * n + j];
			for (l = 0; l < j; ++l)
				value -= a[i * n + l] * a[j * n + l];
			if (!isfinite(value))
				return PME_STATICS_NUMERIC_FAILURE;
			if (i == j)
			{
				if (value <= 64 * DBL_EPSILON * (double)n)
					return PME_STATICS_SINGULAR;
				a[i * n + j] = sqrt(value);
			}
			else
				a[i * n + j] = value / a[j * n + j];
		}
		for (j = 0; j < i; ++j)
			b[i] -= a[i * n + j] * b[j];
		b[i] /= a[i * n + i];
	}
	for (i = n; i-- > 0;)
	{
		for (j = i + 1; j < n; ++j)
			b[i] -= a[j * n + i] * b[j];
		b[i] /= a[i * n + i];
		u[i] = b[i] * scale[i];
		if (!isfinite(u[i]))
			return PME_STATICS_NUMERIC_FAILURE;
	}
	for (i = 0; i < n; ++i)
	{
		double reaction = -m->nodes[i / 3].load[i % 3];
		for (j = 0; j < n; ++j)
			reaction += k[i * n + j] * u[j];
		if (!isfinite(reaction))
			return PME_STATICS_NUMERIC_FAILURE;
		nodes[i / 3].displacement[i % 3] = u[i];
		nodes[i / 3].reaction[i % 3] = reaction;
	}
	for (e = 0; e < m->element_count; ++e)
	{
		double local[36], t[36], displacement[6] = {0};
		size_t dof[6];
		pme_statics_element_result *result = &elements[e];
		(void)element_matrices(m, &m->elements[e], local, t, dof);
		for (i = 0; i < 6; ++i)
			for (j = 0; j < 6; ++j)
				displacement[i] += t[i * 6 + j] * u[dof[j]];
		for (i = 0; i < 6; ++i)
		{
			result->end_force[i] = 0;
			for (j = 0; j < 6; ++j)
				result->end_force[i] += local[i * 6 + j] * displacement[j];
			if (!isfinite(result->end_force[i]))
				return PME_STATICS_NUMERIC_FAILURE;
		}
		result->axial_force = result->end_force[3];
		result->axial_stress = m->elements[e].kind == PME_STATICS_SPRING
								   ? 0
								   : result->axial_force / m->elements[e].area;
		if (!isfinite(result->axial_stress))
			return PME_STATICS_NUMERIC_FAILURE;
	}
	return PME_STATICS_OK;
}
