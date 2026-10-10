#include "YMGUI_Invalidate.h"
#include "YMGUI_Label.h"
#include "demo_host.h"
#include "scene.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define PI 3.14159265358979323846
#define V(x, y, z) ((Vec){x, y, z})
typedef struct {
  double x, y, z;
} Vec;
static Vec add(Vec a, Vec b) { return V(a.x + b.x, a.y + b.y, a.z + b.z); }
static Vec mul(Vec a, double k) { return V(a.x * k, a.y * k, a.z * k); }
static double dot(Vec a, Vec b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
static Vec cross(Vec a, Vec b) {
  return V(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}
static double norm(Vec a) { return sqrt(dot(a, a)); }
static Vec array(const double a[3]) { return V(a[0], a[1], a[2]); }
static Vec unit(Vec a) {
  double n = norm(a);
  return n > 1e-14 ? mul(a, 1 / n) : V(0, 0, 0);
}
typedef struct {
  size_t face, element;
  Vec normal;
  double light;
} SurfaceFace;
typedef struct {
  SolidScene solid;
  double yaw, pitch, zoom, amp;
  Vec center, view_right, view_up, view_eye;
  Vec world[SOLID_NODES], projected[SOLID_NODES];
  SurfaceFace surface[SOLID_FACES];
  size_t surface_count;
  int geometry_valid, cached_cut, cached_slice;
  double cached_amp;
  unsigned solve_count;
  int cut, slice, wire, field, low, shading;
  double vertex_field[3][SOLID_NODES];
} Scene;
static Scene state;
static Vec project(const Scene *s, Vec p) {
  p = add(p, mul(s->center, -1));
  return V(420 + s->zoom * dot(s->view_right, p),
           245 - s->zoom * dot(s->view_up, p), -dot(s->view_eye, p));
}
static void pixel(GRE_RenderTarget t, int x, int y, double z, GRErgb24 c) {
  if (x >= 0 && y >= 0 && x < t->width && y < t->height) {
    size_t i = (size_t)y * t->width + x;
    if (z <= t->zbuff[i]) {
      t->zbuff[i] = (float)z;
      t->data[i] = GRE_FramePixel_From_RGB24(c);
    }
  }
}
static void screen_line(GRE_RenderTarget t, Vec a, Vec b, GRErgb24 color,
                        int width) {
  /* Clip to viewport before rasterization, including greatly amplified shapes.
   */
  double dx = b.x - a.x, dy = b.y - a.y, lo = 0, hi = 1;
  double p[4] = {-dx, dx, -dy, dy},
         q[4] = {a.x + width, t->width + width - 1 - a.x, a.y + width,
                 t->height + width - 1 - a.y};
  for (int i = 0; i < 4; ++i) {
    if (fabs(p[i]) < 1e-15) {
      if (q[i] < 0)
        return;
    } else {
      double r = q[i] / p[i];
      if (p[i] < 0)
        lo = fmax(lo, r);
      else
        hi = fmin(hi, r);
    }
  }
  if (lo > hi)
    return;
  Vec delta = add(b, mul(a, -1));
  b = add(a, mul(delta, hi));
  a = add(a, mul(delta, lo));
  int steps = (int)ceil(fmax(fabs(b.x - a.x), fabs(b.y - a.y)));
  if (!steps)
    steps = 1;
  for (int i = 0; i <= steps; ++i) {
    double f = (double)i / steps;
    Vec v = add(a, mul(add(b, mul(a, -1)), f));
    for (int x = -width; x <= width; ++x)
      for (int y = -width; y <= width; ++y)
        pixel(t, (int)lround(v.x) + x, (int)lround(v.y) + y, v.z, color);
  }
}
static void line(const Scene *s, GRE_RenderTarget t, Vec a, Vec b, GRErgb24 c,
                 int w) {
  screen_line(t, project(s, a), project(s, b), c, w);
}
static const GRErgb24 grey = {94, 110, 129}, gold = {255, 208, 88},
                      green = {75, 225, 168};
static void arrow(const Scene *s, GRE_RenderTarget t, Vec p, Vec force,
                  GRErgb24 color) {
  if (norm(force) < 1e-8)
    return;
  Vec end = add(p, mul(unit(force), .05));
  Vec a = project(s, p), b = project(s, end);
  a.z -= 1e-5;
  b.z -= 1e-5; /* Small surface bias; retain actual depth and occlusion. */
  double d = hypot(b.x - a.x, b.y - a.y);
  screen_line(t, a, b, color, 1);
  if (d < 1)
    return;
  double x = (b.x - a.x) / d, y = (b.y - a.y) / d;
  screen_line(t, b, V(b.x - 9 * x - 4 * y, b.y - 9 * y + 4 * x, b.z), color, 1);
  screen_line(t, b, V(b.x - 9 * x + 4 * y, b.y - 9 * y - 4 * x, b.z), color, 1);
}
static GYOBJ label(GYOBJ parent, int x, int y, int width, const char *text) {
  GYOBJ l = YMGUI_Creat_Label_Creat(parent, x, y, width, 24);
  if (l) {
    YMGUI_Label_SetText(l, text);
    YMGUI_Label_SetTextColor(l, GY_ARGB(255, 222, 232, 243));
  }
  return l;
}
static uint64_t hash(GRE_RenderTarget t) {
  uint64_t h = 14695981039346656037ULL;
  const unsigned char *p = (const unsigned char *)t->data;
  for (size_t i = 0; i < (size_t)t->width * t->height * sizeof(*t->data); ++i)
    h = (h ^ p[i]) * 1099511628211ULL;
  return h;
}
enum {
  LEFT = 1,
  RIGHT = 2,
  UP = 4,
  DOWN = 8,
  IN = 16,
  OUT = 32,
  AMPUP = 64,
  AMPDOWN = 128,
  CUT = 256,
  SLICEUP = 512,
  SLICEDOWN = 1024,
  WIRE = 2048,
  FIELD = 4096,
  LOW = 8192,
  MESH = 16384,
  PATCH = 32768,
  LOADUP = 65536,
  LOADDOWN = 131072,
  RESET = 262144,
  QUIT = 524288,
  SHADING = 1048576
};
static const YMGRE_DemoKeyBinding keys[] = {{'1', 0, 0, 1},
                                            {'2', 0, 0, 2},
                                            {'a', LEFT, 0, 0},
                                            {'d', RIGHT, 0, 0},
                                            {'w', UP, 0, 0},
                                            {'s', DOWN, 0, 0},
                                            {'=', IN, 0, 0},
                                            {'+', IN, 0, 0},
                                            {'-', OUT, 0, 0},
                                            {']', AMPUP, 0, 0},
                                            {'[', AMPDOWN, 0, 0},
                                            {'z', 0, CUT, 0},
                                            {'k', 0, SLICEUP, 0},
                                            {'j', 0, SLICEDOWN, 0},
                                            {'v', 0, WIRE, 0},
                                            {'c', 0, FIELD, 0},
                                            {'h', 0, SHADING, 0},
                                            {'l', 0, LOW, 0},
                                            {'m', 0, MESH, 0},
                                            {'p', 0, PATCH, 0},
                                            {YMGRE_KEY_UP, 0, LOADUP, 0},
                                            {YMGRE_KEY_DOWN, 0, LOADDOWN, 0},
                                            {'r', 0, RESET, 0},
                                            {YMGRE_KEY_ESCAPE, 0, QUIT, 0}};
static Vec node(const Scene *s, size_t i) {
  return add(array(s->solid.nodes[i].position),
             mul(array(s->solid.nr[i].displacement), s->amp));
}
static double maxfield(const Scene *s) {
  return s->field == 0   ? s->solid.max_stress
         : s->field == 1 ? s->solid.max_displacement
                         : s->solid.max_energy;
}
static double field(const Scene *s, size_t e) {
  if (s->field == 0)
    return s->solid.er[e].von_mises;
  if (s->field == 2)
    return s->solid.er[e].energy_density;
  double value = 0;
  for (int i = 0; i < 4; ++i)
    value +=
        norm(array(s->solid.nr[s->solid.tets[e].node[i]].displacement)) / 4;
  return value;
}
/* Display-only volume-weighted recovery. Preserve raw element results and
 * maxima; displacement is already a nodal unknown and needs no averaging. */
static void recover_vertex_fields(Scene *s) {
  double weight[SOLID_NODES] = {0};
  memset(s->vertex_field, 0, sizeof s->vertex_field);
  for (size_t e = 0; e < s->solid.ec; ++e) {
    double v = s->solid.er[e].volume;
    for (int j = 0; j < 4; ++j) {
      size_t n = s->solid.tets[e].node[j];
      weight[n] += v;
      s->vertex_field[0][n] += v * s->solid.er[e].von_mises;
      s->vertex_field[2][n] += v * s->solid.er[e].energy_density;
    }
  }
  for (size_t n = 0; n < s->solid.nc; ++n) {
    if (weight[n] > 0) {
      s->vertex_field[0][n] /= weight[n];
      s->vertex_field[2][n] /= weight[n];
    }
    s->vertex_field[1][n] = norm(array(s->solid.nr[n].displacement));
  }
}
static GRErgb24 heat(double q) {
  q = fmax(0, fmin(1, q));
  double r = fmin(1, fmax(0, 1.5 - fabs(4 * q - 3))),
         g = fmin(1, fmax(0, 1.5 - fabs(4 * q - 2))),
         b = fmin(1, fmax(0, 1.5 - fabs(4 * q - 1)));
  return (GRErgb24){(uint8)(30 + 225 * r), (uint8)(30 + 225 * g),
                    (uint8)(30 + 225 * b)};
}
static GRErgb24 surface_color(const Scene *s, double q, double light) {
  GRErgb24 color = s->shading == 2 ? (GRErgb24){100, 176, 222} : heat(q);
  if (s->low)
    color = q < .1 ? (GRErgb24){85, 225, 225} : (GRErgb24){75, 85, 100};
  color.R = (uint8)(color.R * light);
  color.G = (uint8)(color.G * light);
  color.B = (uint8)(color.B * light);
  return color;
}
static void triangle(const Scene *s, GRE_RenderTarget t, Vec a, Vec b, Vec c,
                     const double q[3], double light) {
  double det = (b.y - c.y) * (a.x - c.x) + (c.x - b.x) * (a.y - c.y);
  if (fabs(det) < 1e-9)
    return;
  int xmin = (int)fmax(0, floor(fmin(a.x, fmin(b.x, c.x)))),
      xmax = (int)fmin(t->width - 1, ceil(fmax(a.x, fmax(b.x, c.x))));
  int ymin = (int)fmax(0, floor(fmin(a.y, fmin(b.y, c.y)))),
      ymax = (int)fmin(t->height - 1, ceil(fmax(a.y, fmax(b.y, c.y))));
  double ux = (b.y - c.y) / det, uy = (c.x - b.x) / det, vx = (c.y - a.y) / det,
         vy = (a.x - c.x) / det;
  double u0 = ux * (xmin + .5 - c.x) + uy * (ymin + .5 - c.y),
         v0 = vx * (xmin + .5 - c.x) + vy * (ymin + .5 - c.y);
  for (int y = ymin; y <= ymax; ++y, u0 += uy, v0 += vy) {
    double u = u0, v = v0;
    for (int x = xmin; x <= xmax; ++x, u += ux, v += vx) {
      double w = 1 - u - v;
      if (u < -1e-8 || v < -1e-8 || w < -1e-8)
        continue;
      double z = u * a.z + v * b.z + w * c.z;
      size_t index = (size_t)y * t->width + x;
      if (z > t->zbuff[index])
        continue;
      t->zbuff[index] = (float)z;
      t->data[index] = GRE_FramePixel_From_RGB24(
          surface_color(s, u * q[0] + v * q[1] + w * q[2], light));
    }
  }
}
static int visible(const Scene *s, size_t e) {
  if (e == SIZE_MAX)
    return 0;
  if (!s->cut)
    return 1;
  double plane = s->solid.length[2] * s->slice / s->solid.div[2];
  for (int i = 0; i < 4; ++i)
    if (s->solid.nodes[s->solid.tets[e].node[i]].position[2] > plane + 1e-10)
      return 0;
  return 1;
}
/* Geometry changes only with a new solution, display amplification or cut.
 * Camera motion transforms each shared vertex once; it never invokes FEM. */
static void prepare_surface(Scene *s) {
  if (s->geometry_valid && s->cached_amp == s->amp && s->cached_cut == s->cut &&
      s->cached_slice == s->slice)
    return;
  for (size_t n = 0; n < s->solid.nc; ++n)
    s->world[n] = node(s, n);
  s->surface_count = 0;
  for (size_t i = 0; i < s->solid.fc; ++i) {
    const SolidFace *f = &s->solid.faces[i];
    int a = visible(s, f->owner), b = visible(s, f->other);
    if (a == b)
      continue;
    size_t e = a ? f->owner : f->other, opposite = SIZE_MAX;
    for (int j = 0; j < 4; ++j) {
      size_t n = s->solid.tets[e].node[j];
      if (n != f->node[0] && n != f->node[1] && n != f->node[2])
        opposite = n;
    }
    Vec ref[3] = {array(s->solid.nodes[f->node[0]].position),
                  array(s->solid.nodes[f->node[1]].position),
                  array(s->solid.nodes[f->node[2]].position)};
    Vec rn = cross(add(ref[1], mul(ref[0], -1)), add(ref[2], mul(ref[0], -1)));
    /* Faces are stored with sorted node IDs, not an oriented winding. Recover
     * the outward direction using the visible tetrahedron's opposite vertex. */
    double sign = dot(rn, add(array(s->solid.nodes[opposite].position),
                              mul(ref[0], -1))) > 0
                      ? -1
                      : 1;
    Vec p = s->world[f->node[0]], q = s->world[f->node[1]],
        r = s->world[f->node[2]];
    SurfaceFace *sf = &s->surface[s->surface_count++];
    sf->face = i;
    sf->element = e;
    sf->normal = mul(unit(cross(add(q, mul(p, -1)), add(r, mul(p, -1)))), sign);
    sf->light = .65 + .35 * fabs(dot(unit(rn), unit(V(.3, .8, -.5))));
  }
  s->cached_amp = s->amp;
  s->cached_cut = s->cut;
  s->cached_slice = s->slice;
  s->geometry_valid = 1;
}
static void load_annotations(Scene *s, GRE_RenderTarget t) {
  double ymin = s->solid.patch ? s->solid.length[1] * .5 : 0,
         zmin = s->solid.patch ? s->solid.length[2] * .5 : 0;
  /* Outline only the physical traction region, never the artificial cut face.
   */
  for (size_t i = 0; i < s->surface_count; ++i) {
    const SurfaceFace *sf = &s->surface[i];
    const SolidFace *f = &s->solid.faces[sf->face];
    if (!SolidScene_FaceLoaded(&s->solid, f) ||
        dot(sf->normal, s->view_eye) <= 0)
      continue;
    for (int j = 0; j < 3; ++j) {
      size_t a = f->node[j], b = f->node[(j + 1) % 3];
      const double *p = s->solid.nodes[a].position,
                   *q = s->solid.nodes[b].position;
      int edge = (fabs(p[1] - ymin) < 1e-10 && fabs(q[1] - ymin) < 1e-10) ||
                 (fabs(p[1] - s->solid.length[1]) < 1e-10 &&
                  fabs(q[1] - s->solid.length[1]) < 1e-10) ||
                 (fabs(p[2] - zmin) < 1e-10 && fabs(q[2] - zmin) < 1e-10) ||
                 (fabs(p[2] - s->solid.length[2]) < 1e-10 &&
                  fabs(q[2] - s->solid.length[2]) < 1e-10);
      if (edge) {
        Vec pa = s->projected[a], pb = s->projected[b];
        pa.z -= 1e-5;
        pb.z -= 1e-5;
        screen_line(t, pa, pb, gold, 1);
      }
    }
  }
  /* Nine equal traction glyphs, independent of mesh density. Interpolate the
   * actual loaded boundary triangles to locate their deformed attachment. */
  for (int iy = 0; iy < 3; ++iy)
    for (int iz = 0; iz < 3; ++iz) {
      double y = ymin + (s->solid.length[1] - ymin) * (iy + .5) / 3,
             z = zmin + (s->solid.length[2] - zmin) * (iz + .5) / 3;
      for (size_t i = 0; i < s->surface_count; ++i) {
        const SurfaceFace *sf = &s->surface[i];
        const SolidFace *f = &s->solid.faces[sf->face];
        if (!SolidScene_FaceLoaded(&s->solid, f))
          continue;
        const double *a = s->solid.nodes[f->node[0]].position,
                     *b = s->solid.nodes[f->node[1]].position,
                     *c = s->solid.nodes[f->node[2]].position;
        double det =
            (b[2] - c[2]) * (a[1] - c[1]) + (c[1] - b[1]) * (a[2] - c[2]);
        double u = ((b[2] - c[2]) * (y - c[1]) + (c[1] - b[1]) * (z - c[2])) /
                   det,
               v = ((c[2] - a[2]) * (y - c[1]) + (a[1] - c[1]) * (z - c[2])) /
                   det,
               w = 1 - u - v;
        if (u < -1e-8 || v < -1e-8 || w < -1e-8)
          continue;
        if (dot(sf->normal, s->view_eye) > 0) {
          Vec anchor = add(
              add(mul(s->world[f->node[0]], u), mul(s->world[f->node[1]], v)),
              mul(s->world[f->node[2]], w));
          anchor = add(anchor, mul(sf->normal, 1e-4));
          Vec force = s->solid.kind == 1 ? V(-1, 0, 0) : V(0, -1, 0);
          arrow(s, t, add(anchor, mul(force, -.05)), force, gold);
        }
        break;
      }
    }
}
static size_t draw(Scene *s, GRE_RenderTarget t) {
  prepare_surface(s);
  double ca = cos(s->yaw), sa = sin(s->yaw), ce = cos(s->pitch),
         se = sin(s->pitch);
  s->view_right = V(ca, 0, -sa);
  s->view_up = V(-se * sa, ce, -se * ca);
  s->view_eye = V(ce * sa, se, ce * ca);
  for (size_t n = 0; n < s->solid.nc; ++n)
    s->projected[n] = project(s, s->world[n]);
  for (size_t i = 0; i < (size_t)t->width * t->height; ++i) {
    t->data[i] = GRE_FramePixel_From_RGB24((GRErgb24){20, 28, 40});
    t->zbuff[i] = 1e20f;
  }
  double max = maxfield(s);
  size_t drawn = 0;
  for (size_t i = 0; i < s->surface_count; ++i) {
    const SurfaceFace *sf = &s->surface[i];
    if (dot(sf->normal, s->view_eye) <= 0)
      continue;
    const SolidFace *f = &s->solid.faces[sf->face];
    double q[3];
    for (int j = 0; j < 3; ++j) {
      double value = s->shading == 0 ? s->vertex_field[s->field][f->node[j]]
                                     : field(s, sf->element);
      q[j] = max > 0 ? value / max : 0;
    }
    Vec pp[3] = {s->projected[f->node[0]], s->projected[f->node[1]],
                 s->projected[f->node[2]]};
    triangle(s, t, pp[0], pp[1], pp[2], q, sf->light);
    if (s->wire)
      for (int j = 0; j < 3; ++j) {
        Vec p = pp[j], r = pp[(j + 1) % 3];
        p.z -= 1e-5;
        r.z -= 1e-5;
        screen_line(t, p, r, (GRErgb24){26, 35, 49}, 0);
      }
    ++drawn;
  }
  /* Undeformed reference bounds and fixed-face boundary, independent of the
   * display cut. */
  for (int i = 0; i < 8; ++i)
    for (int j = 0; j < 3; ++j)
      if (!(i & (1 << j))) {
        int k = i | (1 << j);
        Vec a = V((i & 1) ? s->solid.length[0] : 0,
                  (i & 2) ? s->solid.length[1] : 0,
                  (i & 4) ? s->solid.length[2] : 0),
            b = V((k & 1) ? s->solid.length[0] : 0,
                  (k & 2) ? s->solid.length[1] : 0,
                  (k & 4) ? s->solid.length[2] : 0);
        line(s, t, a, b, grey, 0);
      }
  for (size_t i = 0; i < s->solid.nc; ++i) {
    if (s->solid.nodes[i].fixed &&
        s->solid.nodes[i].position[2] <=
            (s->cut ? s->solid.length[2] * s->slice / s->solid.div[2]
                    : s->solid.length[2]) +
                1e-10) {
      Vec p = project(s, node(s, i));
      p.z -= 1e-5;
      screen_line(t, p, p, green, 1);
    }
  }
  load_annotations(s, t);
  /* Absolute values are listed in the panel; this bar is relative 0..maximum.
   */
  for (int y = 80; (s->shading != 2 || s->low) && y < 380; ++y)
    for (int x = 795; x < 809; ++x)
      pixel(t, x, y, -1e10, heat(1 - (y - 80) / 299.));
  return drawn;
}
static int rebuild(Scene *s, int kind, int fine, int patch, double load,
                   int reset_view) {
  int changed = !s->solve_count || kind != s->solid.kind ||
                fine != s->solid.fine || patch != s->solid.patch ||
                load != s->solid.load;
  if (changed) {
    if (!SolidScene_Build(&s->solid, kind, fine, patch, load) ||
        !SolidScene_Solve(&s->solid)) {
      fprintf(stderr, "Solid solve failed (%d), residual %g\n", s->solid.status,
              s->solid.report.relative_residual);
      return 0;
    }
    ++s->solve_count;
    recover_vertex_fields(s);
    s->geometry_valid = 0;
  }
  if (reset_view) {
    s->yaw = .65; /* Initially show the actual loaded X+ face. */
    s->pitch = .38;
    s->zoom = kind == 1 ? 1400 : 1450;
    s->center = V(s->solid.length[0] / 2, s->solid.length[1] / 2,
                  s->solid.length[2] / 2);
    s->amp = s->solid.amp;
    s->cut = s->field = s->low = 0;
    s->wire = s->shading = 0;
  }
  if (changed || reset_view)
    s->slice = s->solid.div[2] / 2;
  return 1;
}
static int balance(const Scene *s) {
  Vec f = V(0, 0, 0), m = V(0, 0, 0);
  for (size_t i = 0; i < s->solid.nc; ++i) {
    Vec force =
        add(array(s->solid.nodes[i].load), array(s->solid.nr[i].reaction));
    f = add(f, force);
    m = add(m, cross(array(s->solid.nodes[i].position), force));
  }
  return norm(f) < 1e-4 && norm(m) < 1e-4;
}
int main(int argc, char **argv) {
  Scene *s = &state;
  YMGRE_DemoHost host;
  GRE_RenderTarget target = NULL;
  GYOBJ image, rows[14];
  int rc = 1, kind = 1, smoke = 0, initial_cut = 0, initial_shading = 0;
  for (int i = 1; i < argc; ++i) {
    if (!strcmp(argv[i], "--smoke-test"))
      smoke = 1;
    else if (!strcmp(argv[i], "--cut"))
      initial_cut = 1;
    else if (!strcmp(argv[i], "--shading") && i + 1 < argc) {
      const char *mode = argv[++i];
      if (!strcmp(mode, "smooth"))
        initial_shading = 0;
      else if (!strcmp(mode, "element"))
        initial_shading = 1;
      else if (!strcmp(mode, "solid"))
        initial_shading = 2;
      else
        return 1;
    } else if (!strcmp(argv[i], "--scene") && i + 1 < argc) {
      kind = atoi(argv[++i]);
      if (kind != 1 && kind != 2)
        return 1;
    } else
      return 1;
  }
  if (!rebuild(s, kind, 1, 0, 1000, 1))
    return 1;
  s->cut = initial_cut;
  s->shading = initial_shading;
  YMGRE_DemoHost_DefaultScale(1);
  if (!YMGRE_DemoHost_Init(&host, 1200, 760, 40)) {
    fprintf(stderr, "GRE: %s\n", YMGRE_DemoHost_LastError());
    return 1;
  }
  YMGRE_DemoHost_SetTitle(&host,
                          "YMPME | Solid finite elements | cube and block");
  target = YMGRE_Creat_RenderTarget(840, 500);
  if (!target)
    goto done;
  image = YMGRE_DemoHost_AddTarget(&host, target, 10, 60, 840, 500);
  if (!image ||
      !label(host.context->root, 15, 15, 1170,
             "SOLID FEM / TETRAHEDRA     2 examples     Press 1 or 2"))
    goto done;
  for (int i = 0; i < 14; ++i) {
    rows[i] = label(host.context->root, 865, 64 + 35 * i, 325, "");
    if (!rows[i])
      goto done;
  }
  YMGUI_Label_SetTextColor(rows[0], GY_ARGB(255, 255, 208, 88));
  const char *notes[10] = {"1 Cube compression  2 Block bending",
                           "A/D orbit  W/S elevation  +/- zoom",
                           "Z cut  J/K slice position  V mesh edges",
                           "M mesh density  P whole face / load patch",
                           "C stress / displacement / strain energy",
                           "L highlight <10% of current field maximum",
                           "Up/Down load +/-250 N  [/] shape scale",
                           "H smooth / element / solid   R reset  ESC exit",
                           "Low stress alone does not justify removal",
                           "Refine mesh; re-solve after design changes"};
  for (int i = 0; i < 10; ++i)
    if (!label(host.context->root, 15 + 590 * (i % 2), 575 + 33 * (i / 2), 580,
               notes[i]))
      goto done;
  if (!YMGRE_DemoHost_BindKeys(&host, keys, sizeof keys / sizeof keys[0]))
    goto done;
  uint64_t first = 0, smooth_hash = 0, element_hash = 0;
  double verified_stress = 0;
  size_t fullfaces = 0;
  unsigned seen = 0, cached_solves = 0;
  uint64_t orbit_hash = 0;
  double orbit_yaw = 0;
  double last_time = YMGRE_DemoHost_Time();
  size_t faces = 0;
  for (unsigned frame = 0;; ++frame) {
    double now = YMGRE_DemoHost_Time();
    double dt = smoke ? 1. / 60 : fmax(0, fmin(.05, now - last_time));
    last_time = now;
    char text[96];
    if (smoke) {
      int key = frame == 2    ? 'z'
                : frame == 4  ? 'c'
                : frame == 6  ? 'p'
                : frame == 8  ? '2'
                : frame == 10 ? 'm'
                : frame == 12 ? 'l'
                : frame == 14 ? 'l'
                : frame == 16 ? 'h'
                : frame == 18 ? 'h'
                : frame == 20 ? 'h'
                : frame == 30 ? YMGRE_KEY_ESCAPE
                              : 0;
      if (frame == 22 && !YMGRE_DemoHost_InjectKey(&host, 'd', 1))
        goto done;
      if (frame == 27 && !YMGRE_DemoHost_InjectKey(&host, 'd', 0))
        goto done;
      if (key && (!YMGRE_DemoHost_InjectKey(&host, key, 1) ||
                  !YMGRE_DemoHost_InjectKey(&host, key, 0)))
        goto done;
    }
    unsigned a = YMGRE_DemoHost_TakeActions(&host);
    unsigned held = YMGRE_DemoHost_HeldKeys(&host);
    int value = YMGRE_DemoHost_TakeValue(&host);
    if (a & QUIT) {
      rc = smoke && seen != 2047 ? 1 : 0;
      break;
    }
    int reset_view = value || (a & RESET),
        kind1 = value ? value : s->solid.kind, fine = s->solid.fine,
        patch = s->solid.patch;
    double load = s->solid.load;
    if (a & RESET) {
      fine = 1;
      patch = 0;
      load = 1000;
    }
    if (a & MESH)
      fine = !fine;
    if (a & PATCH)
      patch = !patch;
    if (a & LOADUP)
      load += 250;
    if (a & LOADDOWN)
      load -= 250;
    load = fmax(250, fmin(4000, load));
    if (reset_view || (a & (MESH | PATCH | LOADUP | LOADDOWN)))
      if (!rebuild(s, kind1, fine, patch, load, reset_view))
        goto done;
    if (held & LEFT)
      s->yaw -= 1.4 * dt;
    if (held & RIGHT)
      s->yaw += 1.4 * dt;
    if (held & UP)
      s->pitch += 1.2 * dt;
    if (held & DOWN)
      s->pitch -= 1.2 * dt;
    s->pitch = fmax(-1.3, fmin(1.3, s->pitch));
    if (held & IN)
      s->zoom *= exp(1.5 * dt);
    if (held & OUT)
      s->zoom *= exp(-1.5 * dt);
    s->zoom = fmax(400, fmin(4000, s->zoom));
    if (held & AMPUP)
      s->amp *= exp(1.8 * dt);
    if (held & AMPDOWN)
      s->amp *= exp(-1.8 * dt);
    s->amp = fmax(1, fmin(100000, s->amp));
    if (a & CUT)
      s->cut = !s->cut;
    if (a & SLICEUP && s->slice < s->solid.div[2])
      ++s->slice;
    if (a & SLICEDOWN && s->slice > 1)
      --s->slice;
    if (a & WIRE)
      s->wire = !s->wire;
    if (a & SHADING)
      s->shading = (s->shading + 1) % 3;
    if (a & FIELD)
      s->field = (s->field + 1) % 3;
    if (a & LOW)
      s->low = !s->low;
    int dirty = frame == 0 || a || value || held;
    if (dirty) {
      faces = draw(s, target);
      YMGUI_Label_SetText(rows[0], s->solid.kind == 1 ? "1/2  SOLID CUBE"
                                                      : "2/2  SOLID BLOCK");
      snprintf(text, sizeof text, "%zu nodes / %zu tetrahedra", s->solid.nc,
               s->solid.ec);
      YMGUI_Label_SetText(rows[1], text);
      snprintf(text, sizeof text, "Load %.0f N / %s", s->solid.load,
               s->solid.patch ? "patch" : "whole face");
      YMGUI_Label_SetText(rows[2], text);
      snprintf(text, sizeof text, "Max von Mises %.4f MPa",
               s->solid.max_stress / 1e6);
      YMGUI_Label_SetText(rows[3], text);
      snprintf(text, sizeof text, "Max displacement %.5f mm",
               s->solid.max_displacement * 1000);
      YMGUI_Label_SetText(rows[4], text);
      snprintf(text, sizeof text, "Strain energy %.6f J",
               s->solid.report.strain_energy);
      YMGUI_Label_SetText(rows[5], text);
      snprintf(text, sizeof text, "Solves %u / PCG %zu / r %.1e",
               s->solve_count, s->solid.report.iterations,
               s->solid.report.relative_residual);
      YMGUI_Label_SetText(rows[6], text);
      snprintf(text, sizeof text, "Shape x%.0f / cached solution", s->amp);
      YMGUI_Label_SetText(rows[7], text);
      snprintf(text, sizeof text, "Slice %s: %d/%d", s->cut ? "ON" : "OFF",
               s->slice, s->solid.div[2]);
      YMGUI_Label_SetText(rows[8], text);
      YMGUI_Label_SetText(rows[9],
                          s->shading == 2 && !s->low ? "Color: uniform material"
                          : s->field == 0            ? "Color: von Mises stress"
                          : s->field == 1 ? "Color: displacement magnitude"
                                          : "Color: strain energy density");
      snprintf(text, sizeof text, "Raw range 0 .. %.4g %s",
               maxfield(s) / (s->field == 0   ? 1e6
                              : s->field == 1 ? .001
                                              : 1),
               s->field == 0   ? "MPa"
               : s->field == 1 ? "mm"
                               : "J/m3");
      YMGUI_Label_SetText(rows[10], text);
      YMGUI_Label_SetText(rows[11],
                          s->low            ? "Cyan: below 10% (inspect only)"
                          : s->shading == 0 ? "Smooth: averaged nodal values"
                          : s->shading == 1 ? "Raw: constant per tetrahedron"
                                            : "Solid: shape and deformation");
      YMGUI_Label_SetText(rows[12], "Gold: X+ load face / Green: X- fixed");
      YMGUI_Label_SetText(rows[13], "Linear static / no failure model");
      YMGUI_Obj_Invalidate(image);
    }
    if (smoke) {
      if (!balance(s))
        goto done;
      if (frame == 0) {
        first = hash(target);
        fullfaces = faces;
        seen |= 1;
      }
      if (frame == 3 && s->cut && hash(target) != first && faces != fullfaces)
        seen |= 2;
      if (frame == 5 && s->field == 1)
        seen |= 4;
      if (frame == 7 && s->solid.patch)
        seen |= 8;
      if (frame == 9 && s->solid.kind == 2)
        seen |= 16;
      if (frame == 11 && !s->solid.fine)
        seen |= 32;
      if (frame == 13 && s->low)
        seen |= 64;
      if (frame == 15) {
        smooth_hash = hash(target);
        verified_stress = s->solid.max_stress;
      }
      if (frame == 17 && s->shading == 1 && hash(target) != smooth_hash) {
        element_hash = hash(target);
        seen |= 128;
      }
      if (frame == 19 && s->shading == 2 && hash(target) != element_hash)
        seen |= 256;
      if (frame == 21 && s->shading == 0 && hash(target) == smooth_hash &&
          s->solid.max_stress == verified_stress)
        seen |= 512;
      if (frame == 21) {
        cached_solves = s->solve_count;
        orbit_yaw = s->yaw;
        orbit_hash = hash(target);
      }
      if (frame == 28 && s->yaw > orbit_yaw + .05 &&
          hash(target) != orbit_hash && s->solve_count == cached_solves &&
          !held)
        seen |= 1024;
      if (frame > 34)
        goto done;
    }
    if (!YMGRE_DemoHost_Step(&host, smoke ? 0 : 60)) {
      rc = smoke ? 1 : 0;
      break;
    }
  }
done:
  YMGRE_DemoHost_Destroy(&host);
  if (target)
    YMGRE_Free_RenderTarget(target);
  if (smoke)
    printf("%s solid viewer (%u): solve, cut, field, patch, cases, mesh, "
           "low-field display, smooth/raw/solid shading, held orbit with "
           "cached solve\n",
           rc ? "FAIL" : "PASS", seen);
  return rc;
}
