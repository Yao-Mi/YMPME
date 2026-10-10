#include "YMPME_SolidStatics.h"
#include <float.h>
#include <math.h>
#include <stdint.h>
#include <string.h>
size_t pme_solid_workspace_doubles(size_t n, size_t e) {
  size_t limit = SIZE_MAX / sizeof(double);
  if (!n || !e || n > limit / 21 || e > (limit - 21 * n) / 13)
    return 0;
  return 21 * n + 13 * e;
}
pme_solid_options pme_solid_default_options(void) {
  pme_solid_options o = {3000, 1e-8, 1e-9};
  return o;
}
static double dot3(const double *a, const double *b) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
static void cross3(const double *a, const double *b, double *r) {
  r[0] = a[1] * b[2] - a[2] * b[1];
  r[1] = a[2] * b[0] - a[0] * b[2];
  r[2] = a[0] * b[1] - a[1] * b[0];
}
static int fixed(const pme_solid_model *m, size_t i) {
  return (m->nodes[i / 3].fixed & (1u << (i % 3))) != 0;
}
static double vdot(const double *a, const double *b, size_t n) {
  double d = 0;
  for (size_t i = 0; i < n; ++i)
    d += a[i] * b[i];
  return d;
}
static int geometry(const pme_solid_model *m, size_t e, double *g) {
  const pme_solid_tet *t = &m->tets[e];
  double a[3], b[3], c[3], ab[3], bc[3], ca[3];
  for (size_t i = 0; i < 4; ++i) {
    if (t->node[i] >= m->node_count)
      return 0;
    for (size_t j = 0; j < i; ++j)
      if (t->node[i] == t->node[j])
        return 0;
  }
  for (int i = 0; i < 3; ++i) {
    a[i] = m->nodes[t->node[1]].position[i] - m->nodes[t->node[0]].position[i];
    b[i] = m->nodes[t->node[2]].position[i] - m->nodes[t->node[0]].position[i];
    c[i] = m->nodes[t->node[3]].position[i] - m->nodes[t->node[0]].position[i];
  }
  cross3(b, c, bc);
  cross3(c, a, ca);
  cross3(a, b, ab);
  double det = dot3(a, bc), scale = sqrt(dot3(a, a) * dot3(b, b) * dot3(c, c));
  if (!isfinite(det) || !isfinite(scale) || !(scale > 0) ||
      fabs(det) < 1e-10 * scale)
    return 0;
  for (int i = 0; i < 3; ++i) {
    g[3 + i] = bc[i] / det;
    g[6 + i] = ca[i] / det;
    g[9 + i] = ab[i] / det;
    g[i] = -g[3 + i] - g[6 + i] - g[9 + i];
  }
  g[12] = fabs(det) / 6;
  return g[12] > 0;
}
static void strain_stress(const pme_solid_tet *t, const double *g,
                          const double *u, double lambda, double mu,
                          double *eps, double *sig) {
  memset(eps, 0, 6 * sizeof(double));
  for (int a = 0; a < 4; ++a) {
    const double *v = u + 3 * t->node[a], *h = g + 3 * a;
    eps[0] += v[0] * h[0];
    eps[1] += v[1] * h[1];
    eps[2] += v[2] * h[2];
    eps[3] += .5 * (v[0] * h[1] + v[1] * h[0]);
    eps[4] += .5 * (v[1] * h[2] + v[2] * h[1]);
    eps[5] += .5 * (v[2] * h[0] + v[0] * h[2]);
  }
  double tr = eps[0] + eps[1] + eps[2];
  for (int i = 0; i < 6; ++i)
    sig[i] = 2 * mu * eps[i] + (i < 3 ? lambda * tr : 0);
}
static void apply(const pme_solid_model *m, const double *g, const double *u,
                  double *out, double lambda, double mu) {
  memset(out, 0, 3 * m->node_count * sizeof(double));
  for (size_t e = 0; e < m->tet_count; ++e) {
    const double *h = g + 13 * e;
    double eps[6], sig[6];
    strain_stress(&m->tets[e], h, u, lambda, mu, eps, sig);
    for (int a = 0; a < 4; ++a) {
      double *f = out + 3 * m->tets[e].node[a];
      const double *b = h + 3 * a;
      f[0] += h[12] * (sig[0] * b[0] + sig[3] * b[1] + sig[5] * b[2]);
      f[1] += h[12] * (sig[3] * b[0] + sig[1] * b[1] + sig[4] * b[2]);
      f[2] += h[12] * (sig[5] * b[0] + sig[4] * b[1] + sig[2] * b[2]);
    }
  }
}
static int restrained(const pme_solid_model *m) {
  double gram[6][6] = {{0}}, scale = 0;
  const double *origin = m->nodes[0].position;
  for (size_t i = 0; i < m->node_count; ++i)
    for (int j = 0; j < 3; ++j)
      scale = fmax(scale, fabs(m->nodes[i].position[j] - origin[j]));
  if (!(scale > 0))
    return 0;
  for (size_t n = 0; n < m->node_count; ++n) {
    double x = (m->nodes[n].position[0] - origin[0]) / scale,
           y = (m->nodes[n].position[1] - origin[1]) / scale,
           z = (m->nodes[n].position[2] - origin[2]) / scale;
    double rows[3][6] = {
        {1, 0, 0, 0, z, -y}, {0, 1, 0, -z, 0, x}, {0, 0, 1, y, -x, 0}};
    for (int a = 0; a < 3; ++a)
      if (m->nodes[n].fixed & (1u << a))
        for (int i = 0; i < 6; ++i)
          for (int j = 0; j < 6; ++j)
            gram[i][j] += rows[a][i] * rows[a][j];
  }
  double maximum = 0;
  for (int i = 0; i < 6; ++i)
    maximum = fmax(maximum, gram[i][i]);
  for (int j = 0; j < 6; ++j) {
    int best = j;
    for (int i = j + 1; i < 6; ++i)
      if (fabs(gram[i][j]) > fabs(gram[best][j]))
        best = i;
    if (fabs(gram[best][j]) <= maximum * 1e-12)
      return 0;
    for (int i = j; i < 6; ++i) {
      double tmp = gram[j][i];
      gram[j][i] = gram[best][i];
      gram[best][i] = tmp;
    }
    for (int i = j + 1; i < 6; ++i) {
      double f = gram[i][j] / gram[j][j];
      for (int k = j; k < 6; ++k)
        gram[i][k] -= f * gram[j][k];
    }
  }
  return 1;
}
pme_solid_status pme_solid_solve(const pme_solid_model *m,
                                 pme_solid_node_result *nr, size_t nc,
                                 pme_solid_tet_result *er, size_t ec,
                                 const pme_solid_options *options,
                                 pme_solid_report *report, double *w,
                                 size_t wc) {
  if (report)
    memset(report, 0, sizeof(*report));
  if (!m || !m->nodes || !m->tets || !nr || !er || !w)
    return PME_SOLID_INVALID;
  size_t need = pme_solid_workspace_doubles(m->node_count, m->tet_count),
         n = 3 * m->node_count;
  if (!need || wc < need || nc < m->node_count || ec < m->tet_count)
    return PME_SOLID_CAPACITY;
  pme_solid_options o = options ? *options : pme_solid_default_options();
  if (!(o.relative_tolerance > 0 && o.relative_tolerance < 1) ||
      !isfinite(o.absolute_tolerance) || o.absolute_tolerance < 0 ||
      !o.max_iterations || !(m->young_modulus > 0) ||
      !isfinite(m->young_modulus) ||
      !(m->poisson_ratio > -1 && m->poisson_ratio < .49))
    return PME_SOLID_INVALID;
  double lambda = m->young_modulus * m->poisson_ratio /
                  ((1 + m->poisson_ratio) * (1 - 2 * m->poisson_ratio)),
         mu = m->young_modulus / (2 * (1 + m->poisson_ratio));
  if (!isfinite(lambda) || !isfinite(mu) || !(mu > 0))
    return PME_SOLID_NUMERIC;
  for (size_t i = 0; i < m->node_count; ++i) {
    if (m->nodes[i].fixed & ~7u)
      return PME_SOLID_INVALID;
    for (int j = 0; j < 3; ++j)
      if (!isfinite(m->nodes[i].position[j]) ||
          !isfinite(m->nodes[i].load[j]) ||
          !isfinite(m->nodes[i].prescribed[j]) ||
          (!(m->nodes[i].fixed & (1u << j)) && m->nodes[i].prescribed[j] != 0))
        return PME_SOLID_INVALID;
  }
  double *g = w, *u = g + 13 * m->tet_count, *r = u + n, *z = r + n, *p = z + n,
         *ap = p + n, *diag = ap + n, *b = diag + n;
  memset(diag, 0, n * sizeof(double));
  memset(p, 0, n * sizeof(double));
  p[0] = 1;
  for (size_t e = 0; e < m->tet_count; ++e) {
    double *h = g + 13 * e;
    if (!geometry(m, e, h))
      return PME_SOLID_INVALID;
    for (int a = 0; a < 4; ++a)
      for (int j = 0; j < 3; ++j) {
        double *v = h + 3 * a;
        diag[3 * m->tets[e].node[a] + j] +=
            h[12] * (mu * dot3(v, v) + (lambda + mu) * v[j] * v[j]);
      }
  }
  /* Reject disconnected meshes and unused nodes, using temporary vector
   * storage. */
  size_t reached = 1, old = 0;
  while (reached != old) {
    old = reached;
    for (size_t e = 0; e < m->tet_count; ++e) {
      int hit = 0;
      for (int a = 0; a < 4; ++a)
        if (p[3 * m->tets[e].node[a]])
          hit = 1;
      if (hit)
        for (int a = 0; a < 4; ++a) {
          size_t i = 3 * m->tets[e].node[a];
          if (!p[i]) {
            p[i] = 1;
            ++reached;
          }
        }
    }
  }
  if (reached != m->node_count)
    return PME_SOLID_INVALID;
  if (!restrained(m))
    return PME_SOLID_UNRESTRAINED;
  for (size_t i = 0; i < n; ++i) {
    if (!(diag[i] > 0) || !isfinite(diag[i]))
      return PME_SOLID_NUMERIC;
    u[i] = fixed(m, i) ? m->nodes[i / 3].prescribed[i % 3] : 0;
  }
  apply(m, g, u, ap, lambda, mu);
  for (size_t i = 0; i < n; ++i) {
    b[i] = fixed(m, i) ? 0 : m->nodes[i / 3].load[i % 3] - ap[i];
    r[i] = b[i];
    z[i] = r[i] / diag[i];
    p[i] = z[i];
  }
  double reference = sqrt(vdot(b, b, n)), residual = reference,
         tolerance =
             fmax(o.absolute_tolerance, o.relative_tolerance * reference),
         rz = vdot(r, z, n);
  size_t iteration = 0;
  if (!isfinite(reference) || !isfinite(rz))
    return PME_SOLID_NUMERIC;
  while (residual > tolerance && iteration < o.max_iterations) {
    apply(m, g, p, ap, lambda, mu);
    for (size_t i = 0; i < n; ++i)
      if (fixed(m, i))
        ap[i] = 0;
    double pap = vdot(p, ap, n);
    if (!(pap > 0) || !isfinite(pap))
      return PME_SOLID_NUMERIC;
    double alpha = rz / pap;
    for (size_t i = 0; i < n; ++i) {
      u[i] += alpha * p[i];
      r[i] -= alpha * ap[i];
    }
    ++iteration;
    residual = sqrt(vdot(r, r, n));
    if (!isfinite(residual))
      return PME_SOLID_NUMERIC;
    /* Verify true residual before accepting; periodic restart limits residual
     * drift. */
    int restart = residual <= tolerance || iteration % 1000 == 0;
    if (restart) {
      apply(m, g, u, ap, lambda, mu);
      for (size_t i = 0; i < n; ++i)
        r[i] = fixed(m, i) ? 0 : m->nodes[i / 3].load[i % 3] - ap[i];
      residual = sqrt(vdot(r, r, n));
    }
    if (residual <= tolerance)
      break;
    for (size_t i = 0; i < n; ++i)
      z[i] = r[i] / diag[i];
    double next = vdot(r, z, n);
    if (!(next > 0) || !isfinite(next))
      return PME_SOLID_NUMERIC;
    double beta = restart ? 0 : next / rz;
    for (size_t i = 0; i < n; ++i)
      p[i] = z[i] + beta * p[i];
    rz = next;
  }
  if (report) {
    report->iterations = iteration;
    report->residual_norm = residual;
    report->relative_residual = reference > 0 ? residual / reference : 0;
  }
  if (residual > tolerance)
    return PME_SOLID_NO_CONVERGENCE;
  apply(m, g, u, ap, lambda, mu);
  for (size_t i = 0; i < n; ++i) {
    if (!isfinite(u[i]) || !isfinite(ap[i]))
      return PME_SOLID_NUMERIC;
    nr[i / 3].displacement[i % 3] = u[i];
    nr[i / 3].reaction[i % 3] = ap[i] - m->nodes[i / 3].load[i % 3];
  }
  double total = 0;
  for (size_t e = 0; e < m->tet_count; ++e) {
    pme_solid_tet_result *out = &er[e];
    strain_stress(&m->tets[e], g + 13 * e, u, lambda, mu, out->strain,
                  out->stress);
    const double *s = out->stress;
    out->volume = g[13 * e + 12];
    out->von_mises = sqrt(.5 * ((s[0] - s[1]) * (s[0] - s[1]) +
                                (s[1] - s[2]) * (s[1] - s[2]) +
                                (s[2] - s[0]) * (s[2] - s[0])) +
                          3 * (s[3] * s[3] + s[4] * s[4] + s[5] * s[5]));
    out->energy_density = 0;
    for (int i = 0; i < 6; ++i) {
      if (!isfinite(s[i]) || !isfinite(out->strain[i]))
        return PME_SOLID_NUMERIC;
      out->energy_density += .5 * s[i] * out->strain[i] * (i < 3 ? 1 : 2);
    }
    if (!isfinite(out->von_mises) || !isfinite(out->energy_density))
      return PME_SOLID_NUMERIC;
    total += out->energy_density * out->volume;
  }
  if (!isfinite(total))
    return PME_SOLID_NUMERIC;
  if (report)
    report->strain_energy = total;
  return PME_SOLID_OK;
}
