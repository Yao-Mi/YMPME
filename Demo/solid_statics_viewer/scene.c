#include "scene.h"
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
static size_t index3(const SolidScene *s, int x, int y, int z) {
  return (size_t)((z * (s->div[1] + 1) + y) * (s->div[0] + 1) + x);
}
static int facecmp(const void *aa, const void *bb) {
  const SolidFace *a = aa, *b = bb;
  for (int i = 0; i < 3; ++i) {
    if (a->node[i] < b->node[i])
      return -1;
    if (a->node[i] > b->node[i])
      return 1;
  }
  return 0;
}
int SolidScene_FaceLoaded(const SolidScene *s, const SolidFace *f) {
  if (f->other != SIZE_MAX)
    return 0;
  for (int j = 0; j < 3; ++j) {
    const double *p = s->nodes[f->node[j]].position;
    if (fabs(p[0] - s->length[0]) > 1e-10)
      return 0;
    if (s->patch &&
        (p[1] < s->length[1] * .5 - 1e-10 || p[2] < s->length[2] * .5 - 1e-10))
      return 0;
  }
  return 1;
}
int SolidScene_Build(SolidScene *s, int kind, int fine, int patch,
                     double load) {
  memset(s, 0, sizeof(*s));
  s->kind = kind;
  s->fine = fine;
  s->patch = patch;
  s->load = load;
  s->length[0] = kind == 1 ? .2 : .4;
  s->length[1] = s->length[2] = kind == 1 ? .2 : .12;
  s->amp = kind == 1 ? 15000 : 1000;
  s->div[0] = kind == 1 ? (fine ? 8 : 4) : (fine ? 12 : 6);
  s->div[1] = s->div[2] = kind == 1 ? (fine ? 8 : 4) : (fine ? 4 : 2);
  s->nc = (size_t)(s->div[0] + 1) * (s->div[1] + 1) * (s->div[2] + 1);
  for (int z = 0; z <= s->div[2]; ++z)
    for (int y = 0; y <= s->div[1]; ++y)
      for (int x = 0; x <= s->div[0]; ++x) {
        pme_solid_node *n = &s->nodes[index3(s, x, y, z)];
        int pos[3] = {x, y, z};
        for (int j = 0; j < 3; ++j)
          n->position[j] = s->length[j] * pos[j] / s->div[j];
        if (x == 0)
          n->fixed = 7;
      }
  const int tet[6][4] = {{0, 1, 3, 7}, {0, 3, 2, 7}, {0, 2, 6, 7},
                         {0, 6, 4, 7}, {0, 4, 5, 7}, {0, 5, 1, 7}};
  for (int z = 0; z < s->div[2]; ++z)
    for (int y = 0; y < s->div[1]; ++y)
      for (int x = 0; x < s->div[0]; ++x) {
        size_t corners[8];
        for (int i = 0; i < 8; ++i)
          corners[i] =
              index3(s, x + (i & 1), y + ((i >> 1) & 1), z + ((i >> 2) & 1));
        for (int i = 0; i < 6; ++i) {
          for (int j = 0; j < 4; ++j)
            s->tets[s->ec].node[j] = corners[tet[i][j]];
          ++s->ec;
        }
      }
  for (size_t e = 0; e < s->ec; ++e)
    for (int opposite = 0; opposite < 4; ++opposite) {
      SolidFace *f = &s->faces[s->fc++];
      int k = 0;
      for (int j = 0; j < 4; ++j)
        if (j != opposite)
          f->node[k++] = s->tets[e].node[j];
      for (int a = 0; a < 3; ++a)
        for (int b = a + 1; b < 3; ++b)
          if (f->node[b] < f->node[a]) {
            size_t v = f->node[a];
            f->node[a] = f->node[b];
            f->node[b] = v;
          }
      f->owner = e;
      f->other = SIZE_MAX;
    }
  qsort(s->faces, s->fc, sizeof(*s->faces), facecmp);
  size_t count = 0;
  for (size_t i = 0; i < s->fc;) {
    SolidFace f = s->faces[i++];
    if (i < s->fc && !facecmp(&f, &s->faces[i]))
      f.other = s->faces[i++].owner;
    if (i < s->fc && !facecmp(&f, &s->faces[i]))
      return 0;
    s->faces[count++] = f;
  }
  s->fc = count;
  double area = 0;
  for (size_t i = 0; i < s->fc; ++i) {
    SolidFace *f = &s->faces[i];
    if (!SolidScene_FaceLoaded(s, f))
      continue;
    const double *a = s->nodes[f->node[0]].position,
                 *b = s->nodes[f->node[1]].position,
                 *c = s->nodes[f->node[2]].position;
    double ar =
        fabs((b[1] - a[1]) * (c[2] - a[2]) - (b[2] - a[2]) * (c[1] - a[1])) / 2;
    area += ar;
    for (int j = 0; j < 3; ++j)
      s->nodes[f->node[j]].load[kind == 1 ? 0 : 1] -= ar / 3;
  }
  if (!(area > 0))
    return 0;
  for (size_t i = 0; i < s->nc; ++i)
    for (int j = 0; j < 3; ++j)
      s->nodes[i].load[j] *= load / area;
  return 1;
}
int SolidScene_Solve(SolidScene *s) {
  pme_solid_model m = {s->nodes, s->nc, s->tets, s->ec, 70e9, .3};
  pme_solid_options options = pme_solid_default_options();
  s->status = pme_solid_solve(&m, s->nr, SOLID_NODES, s->er, SOLID_TETS,
                              &options, &s->report, s->workspace, SOLID_WORK);
  if (s->status != PME_SOLID_OK)
    return 0;
  s->max_displacement = s->max_stress = s->max_energy = 0;
  for (size_t i = 0; i < s->nc; ++i) {
    double *u = s->nr[i].displacement;
    s->max_displacement = fmax(s->max_displacement,
                               sqrt(u[0] * u[0] + u[1] * u[1] + u[2] * u[2]));
  }
  for (size_t i = 0; i < s->ec; ++i) {
    s->max_stress = fmax(s->max_stress, s->er[i].von_mises);
    s->max_energy = fmax(s->max_energy, s->er[i].energy_density);
  }
  return 1;
}
