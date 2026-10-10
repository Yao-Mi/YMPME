#include "YMPME_Statics3D.h"
#include <float.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

size_t pme_statics3d_workspace_doubles(size_t node_count) {
  size_t n, limit = SIZE_MAX / sizeof(double);
  if (!node_count || node_count > limit / 6)
    return 0;
  n = 6 * node_count;
  if (n > limit / 3 || n > (limit - 3 * n) / 2 / n)
    return 0;
  return 2 * n * n + 3 * n;
}
static double norm3(const double v[3]) {
  return hypot(hypot(v[0], v[1]), v[2]);
}
static void cross3(const double a[3], const double b[3], double c[3]) {
  c[0] = a[1] * b[2] - a[2] * b[1];
  c[1] = a[2] * b[0] - a[0] * b[2];
  c[2] = a[0] * b[1] - a[1] * b[0];
}
pme_statics_status pme_statics3d_element_frame(const pme_statics3d_model *m,
                                               const pme_statics3d_element *e,
                                               double axes[9]) {
  double x[3], y[3], z[3], length;
  size_t i;
  if (!m || !m->nodes || !e || !axes || e->node_a >= m->node_count ||
      e->node_b >= m->node_count || e->node_a == e->node_b ||
      e->kind < PME_STATICS_SPRING || e->kind > PME_STATICS_BEAM)
    return PME_STATICS_INVALID_ARGUMENT;
  for (i = 0; i < 3; ++i)
    x[i] = m->nodes[e->node_b].position[i] - m->nodes[e->node_a].position[i];
  length = norm3(x);
  if (!(length > 0) || !isfinite(length))
    return PME_STATICS_INVALID_ARGUMENT;
  for (i = 0; i < 3; ++i)
    x[i] /= length;
  if (e->kind == PME_STATICS_BEAM) {
    for (i = 0; i < 3; ++i)
      y[i] = e->local_y[i];
    length = norm3(y);
    if (!(length > 0) || !isfinite(length))
      return PME_STATICS_INVALID_ARGUMENT;
    for (i = 0; i < 3; ++i)
      y[i] /= length;
  } else {
    size_t best = 0;
    for (i = 1; i < 3; ++i)
      if (fabs(x[i]) < fabs(x[best]))
        best = i;
    for (i = 0; i < 3; ++i)
      y[i] = i == best ? 1 : 0;
  }
  cross3(x, y, z);
  length = norm3(z);
  if (!(length > 1e-10) || !isfinite(length))
    return PME_STATICS_INVALID_ARGUMENT;
  for (i = 0; i < 3; ++i)
    z[i] /= length;
  cross3(z, x, y);
  for (i = 0; i < 3; ++i) {
    axes[i] = x[i];
    axes[3 + i] = y[i];
    axes[6 + i] = z[i];
  }
  return PME_STATICS_OK;
}
static void pair_stiffness(double k[144], size_t a, size_t b, double value) {
  k[a * 12 + a] += value;
  k[b * 12 + b] += value;
  k[a * 12 + b] -= value;
  k[b * 12 + a] -= value;
}
static void bending_stiffness(double k[144], const size_t dof[4], double ei,
                              double l, double sign) {
  double h = ei / l, b = 6 * h / l, v = 12 * h / l / l;
  double a[16] = {v,  b,  -v, b,  b, 4 * h, -b, 2 * h,
                  -v, -b, v,  -b, b, 2 * h, -b, 4 * h};
  for (size_t i = 0; i < 4; ++i)
    for (size_t j = 0; j < 4; ++j)
      k[dof[i] * 12 + dof[j]] +=
          a[i * 4 + j] * (i % 2 ? sign : 1) * (j % 2 ? sign : 1);
}
static pme_statics_status element_matrices(const pme_statics3d_model *m,
                                           const pme_statics3d_element *e,
                                           double k[144], double t[144],
                                           size_t dof[12]) {
  double axes[9], delta[3], length, axial;
  pme_statics_status status = pme_statics3d_element_frame(m, e, axes);
  if (status != PME_STATICS_OK)
    return status;
  for (size_t i = 0; i < 3; ++i)
    delta[i] =
        m->nodes[e->node_b].position[i] - m->nodes[e->node_a].position[i];
  length = norm3(delta);
  memset(k, 0, 144 * sizeof(double));
  memset(t, 0, 144 * sizeof(double));
  for (size_t b = 0; b < 4; ++b)
    for (size_t i = 0; i < 3; ++i)
      for (size_t j = 0; j < 3; ++j)
        t[(3 * b + i) * 12 + 3 * b + j] = axes[i * 3 + j];
  for (size_t i = 0; i < 12; ++i)
    dof[i] = 6 * (i < 6 ? e->node_a : e->node_b) + i % 6;
  if (e->kind == PME_STATICS_SPRING)
    axial = e->stiffness;
  else {
    if (!(e->young_modulus > 0) || !isfinite(e->young_modulus) ||
        !(e->area > 0) || !isfinite(e->area))
      return PME_STATICS_INVALID_ARGUMENT;
    axial = e->young_modulus * (e->area / length);
  }
  if (e->kind == PME_STATICS_SPRING && (!(axial > 0) || !isfinite(axial)))
    return PME_STATICS_INVALID_ARGUMENT;
  if (!(axial > 0) || !isfinite(axial))
    return PME_STATICS_NUMERIC_FAILURE;
  pair_stiffness(k, 0, 6, axial);
  if (e->kind == PME_STATICS_BEAM) {
    const size_t bend_y[4] = {1, 5, 7, 11}, bend_z[4] = {2, 4, 8, 10};
    double params[4] = {e->shear_modulus, e->torsion_constant,
                        e->second_moment_y, e->second_moment_z};
    for (size_t i = 0; i < 4; ++i)
      if (!(params[i] > 0) || !isfinite(params[i]))
        return PME_STATICS_INVALID_ARGUMENT;
    pair_stiffness(k, 3, 9, e->shear_modulus * (e->torsion_constant / length));
    bending_stiffness(k, bend_y, e->young_modulus * e->second_moment_z, length,
                      1);
    /* Positive rotation about local y slopes the local z displacement
     * negatively. */
    bending_stiffness(k, bend_z, e->young_modulus * e->second_moment_y, length,
                      -1);
  }
  for (size_t i = 0; i < 144; ++i)
    if (!isfinite(k[i]))
      return PME_STATICS_NUMERIC_FAILURE;
  if (e->kind == PME_STATICS_BEAM &&
      (!(k[39] > 0) || !(k[13] > 0) || !(k[26] > 0)))
    return PME_STATICS_NUMERIC_FAILURE;
  return PME_STATICS_OK;
}

static int fixed_dof(const pme_statics3d_model *m, size_t i, double diagonal) {
  const pme_statics3d_node *node = &m->nodes[i / 6];
  return (node->fixed & (1u << (i % 6))) ||
         (i % 6 >= 3 && diagonal == 0 && node->load[i % 6] == 0);
}

pme_statics_status pme_statics3d_solve(
    const pme_statics3d_model *m, pme_statics3d_node_result *nodes,
    size_t node_capacity, pme_statics3d_element_result *elements,
    size_t element_capacity, double *workspace, size_t workspace_doubles) {
  size_t needed, n, i, j, l, e;
  double *k, *a, *b, *u, *scale;
  if (!m || !m->nodes || !m->node_count || !nodes || !workspace ||
      (m->element_count && (!m->elements || !elements)))
    return PME_STATICS_INVALID_ARGUMENT;
  needed = pme_statics3d_workspace_doubles(m->node_count);
  if (!needed || workspace_doubles < needed || node_capacity < m->node_count ||
      element_capacity < m->element_count)
    return PME_STATICS_CAPACITY_EXCEEDED;
  n = m->node_count * 6;
  k = workspace;
  a = k + n * n;
  b = a + n * n;
  u = b + n;
  scale = u + n;
  for (i = 0; i < m->node_count; ++i) {
    const pme_statics3d_node *node = &m->nodes[i];
    if (!isfinite(node->position[0]) || !isfinite(node->position[1]) ||
        !isfinite(node->position[2]) || (node->fixed & ~63u))
      return PME_STATICS_INVALID_ARGUMENT;
    for (j = 0; j < 6; ++j)
      if (!isfinite(node->load[j]) || !isfinite(node->prescribed[j]) ||
          (!(node->fixed & (1u << j)) && node->prescribed[j] != 0))
        return PME_STATICS_INVALID_ARGUMENT;
  }
  memset(k, 0, n * n * sizeof(double));
  for (e = 0; e < m->element_count; ++e) {
    double local[144], t[144], kt[144] = {0};
    size_t dof[12];
    pme_statics_status status =
        element_matrices(m, &m->elements[e], local, t, dof);
    if (status != PME_STATICS_OK)
      return status;
    for (i = 0; i < 12; ++i)
      for (j = 0; j < 12; ++j)
        for (l = 0; l < 12; ++l)
          kt[i * 12 + j] += local[i * 12 + l] * t[l * 12 + j];
    for (i = 0; i < 12; ++i)
      for (j = 0; j < 12; ++j)
        for (l = 0; l < 12; ++l)
          k[dof[i] * n + dof[j]] += t[l * 12 + i] * kt[l * 12 + j];
  }
  for (i = 0; i < n * n; ++i)
    if (!isfinite(k[i]))
      return PME_STATICS_NUMERIC_FAILURE;
  /* Eliminate prescribed values and equilibrate units before Cholesky. */
  for (i = 0; i < n; ++i) {
    int fixed = fixed_dof(m, i, k[i * n + i]);
    u[i] = fixed ? m->nodes[i / 6].prescribed[i % 6] : 0;
    if (!fixed && !(k[i * n + i] > 0))
      return PME_STATICS_SINGULAR;
    scale[i] = fixed ? 1 : 1 / sqrt(k[i * n + i]);
  }
  for (i = 0; i < n; ++i) {
    int fixed = fixed_dof(m, i, k[i * n + i]);
    b[i] = fixed ? u[i] : m->nodes[i / 6].load[i % 6];
    for (j = 0; j < n; ++j) {
      int other_fixed = fixed_dof(m, j, k[j * n + j]);
      if (!fixed && other_fixed)
        b[i] -= k[i * n + j] * u[j];
      a[i * n + j] = fixed || other_fixed
                         ? (i == j ? 1 : 0)
                         : (k[i * n + j] * scale[i]) * scale[j];
    }
    b[i] *= scale[i];
    if (!isfinite(b[i]))
      return PME_STATICS_NUMERIC_FAILURE;
  }
  for (i = 0; i < n; ++i) {
    for (j = 0; j <= i; ++j) {
      double value = a[i * n + j];
      for (l = 0; l < j; ++l)
        value -= a[i * n + l] * a[j * n + l];
      if (!isfinite(value))
        return PME_STATICS_NUMERIC_FAILURE;
      if (i == j) {
        if (value <= 64 * DBL_EPSILON * (double)n)
          return PME_STATICS_SINGULAR;
        a[i * n + j] = sqrt(value);
      } else
        a[i * n + j] = value / a[j * n + j];
    }
    for (j = 0; j < i; ++j)
      b[i] -= a[i * n + j] * b[j];
    b[i] /= a[i * n + i];
  }
  for (i = n; i-- > 0;) {
    for (j = i + 1; j < n; ++j)
      b[i] -= a[j * n + i] * b[j];
    b[i] /= a[i * n + i];
    u[i] = b[i] * scale[i];
    if (!isfinite(u[i]))
      return PME_STATICS_NUMERIC_FAILURE;
  }
  for (i = 0; i < n; ++i) {
    double reaction = -m->nodes[i / 6].load[i % 6];
    for (j = 0; j < n; ++j)
      reaction += k[i * n + j] * u[j];
    if (!isfinite(reaction))
      return PME_STATICS_NUMERIC_FAILURE;
    nodes[i / 6].displacement[i % 6] = u[i];
    nodes[i / 6].reaction[i % 6] = reaction;
  }
  for (e = 0; e < m->element_count; ++e) {
    double local[144], t[144], displacement[12] = {0};
    size_t dof[12];
    pme_statics3d_element_result *result = &elements[e];
    (void)element_matrices(m, &m->elements[e], local, t, dof);
    for (i = 0; i < 12; ++i)
      for (j = 0; j < 12; ++j)
        displacement[i] += t[i * 12 + j] * u[dof[j]];
    for (i = 0; i < 12; ++i) {
      result->end_force[i] = 0;
      for (j = 0; j < 12; ++j)
        result->end_force[i] += local[i * 12 + j] * displacement[j];
      if (!isfinite(result->end_force[i]))
        return PME_STATICS_NUMERIC_FAILURE;
    }
    result->axial_force = result->end_force[6];
    result->axial_stress = m->elements[e].kind == PME_STATICS_SPRING
                               ? 0
                               : result->axial_force / m->elements[e].area;
    if (!isfinite(result->axial_stress))
      return PME_STATICS_NUMERIC_FAILURE;
  }
  return PME_STATICS_OK;
}
