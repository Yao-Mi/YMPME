#include "STATICS/YMPME_Statics3D.h"
#include "YMGUI_Invalidate.h"
#include "YMGUI_Label.h"
#include "demo_host.h"
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
static Vec rotate(Vec v, Vec w) {
  double a = norm(w);
  if (a < 1e-12)
    return v;
  w = mul(w, 1 / a);
  return add(add(mul(v, cos(a)), mul(cross(w, v), sin(a))),
             mul(w, dot(w, v) * (1 - cos(a))));
}
enum {
  FXP = 1,
  FXM = 2,
  FYP = 4,
  FYM = 8,
  FZP = 16,
  FZM = 32,
  MXP = 64,
  MXM = 128,
  YAWP = 256,
  YAWM = 512,
  PITCHP = 1024,
  PITCHM = 2048,
  ZOOMP = 4096,
  ZOOMM = 8192,
  AMPP = 16384,
  AMPM = 32768,
  NEXT = 65536,
  RESET = 131072,
  CAMERA = 262144,
  QUIT = 524288
};
static const YMGRE_DemoKeyBinding keys[] = {{'1', 0, 0, 1},
                                            {'2', 0, 0, 2},
                                            {'3', 0, 0, 3},
                                            {YMGRE_KEY_RIGHT, 0, FXP, 0},
                                            {YMGRE_KEY_LEFT, 0, FXM, 0},
                                            {YMGRE_KEY_UP, 0, FYP, 0},
                                            {YMGRE_KEY_DOWN, 0, FYM, 0},
                                            {'e', 0, FZP, 0},
                                            {'q', 0, FZM, 0},
                                            {'t', 0, MXP, 0},
                                            {'g', 0, MXM, 0},
                                            {'d', 0, YAWP, 0},
                                            {'a', 0, YAWM, 0},
                                            {'w', 0, PITCHP, 0},
                                            {'s', 0, PITCHM, 0},
                                            {'=', 0, ZOOMP, 0},
                                            {'+', 0, ZOOMP, 0},
                                            {'-', 0, ZOOMM, 0},
                                            {']', 0, AMPP, 0},
                                            {'[', 0, AMPM, 0},
                                            {YMGRE_KEY_TAB, 0, NEXT, 0},
                                            {'r', 0, RESET, 0},
                                            {'c', 0, CAMERA, 0},
                                            {YMGRE_KEY_ESCAPE, 0, QUIT, 0}};
typedef struct {
  pme_statics3d_node nodes[5];
  pme_statics3d_element elements[4];
  pme_statics3d_node_result result[5];
  pme_statics3d_element_result force[4];
  double work[1890];
  size_t nc, ec, loaded, selected;
  int scene;
  double amp, yaw, pitch, zoom;
  Vec center;
} Scene;
static void camera_reset(Scene *s) {
  s->yaw = -.6;
  s->pitch = .4;
  s->zoom = 190;
}
static void reset(Scene *s, int scene) {
  memset(s, 0, sizeof(*s));
  s->scene = scene;
  camera_reset(s);
  if (scene == 2) {
    s->nc = 5;
    s->ec = 4;
    s->loaded = 4;
    s->amp = 1200;
    s->center = V(0, .4, 0);
    for (size_t i = 0; i < 4; ++i) {
      s->nodes[i].position[0] = (i & 1) ? 1 : -1;
      s->nodes[i].position[2] = (i & 2) ? 1 : -1;
      s->nodes[i].fixed = 7;
      s->elements[i].kind = PME_STATICS_TRUSS;
      s->elements[i].node_a = i;
      s->elements[i].node_b = 4;
      s->elements[i].young_modulus = 200e9;
      s->elements[i].area = 1e-6;
    }
    s->nodes[4].position[1] = 1;
    s->nodes[4].load[1] = -10;
  } else {
    pme_statics3d_element *e = &s->elements[0];
    double r = .015;
    s->nc = 2;
    s->ec = 1;
    s->loaded = 1;
    s->amp = scene == 3 ? 500 : 80;
    s->center = V(1, 0, 0);
    s->nodes[0].fixed = 63;
    s->nodes[1].position[0] = 2;
    e->kind = PME_STATICS_BEAM;
    e->node_b = 1;
    e->young_modulus = 200e9;
    e->area = PI * r * r;
    e->shear_modulus = 80e9;
    e->second_moment_y = e->second_moment_z = PI * r * r * r * r / 4;
    e->torsion_constant = 2 * e->second_moment_y;
    e->local_y[1] = 1;
    if (scene == 1) {
      s->nodes[1].load[1] = -10;
      s->nodes[1].load[2] = 5;
    } else
      s->nodes[1].load[3] = 3;
  }
  s->selected = s->loaded;
}
static int solve(Scene *s) {
  pme_statics3d_model m = {s->nodes, s->nc, s->elements, s->ec};
  return pme_statics3d_solve(&m, s->result, 5, s->force, 4, s->work, 1890) == 0;
}
/* Fixed orthographic camera; only camera keys change this transform. */
static Vec project(const Scene *s, Vec p) {
  p = add(p, mul(s->center, -1));
  double ca = cos(s->yaw), sa = sin(s->yaw), ce = cos(s->pitch),
         se = sin(s->pitch);
  return V(420 + s->zoom * (ca * p.x - sa * p.z),
           245 - s->zoom * (-se * sa * p.x + ce * p.y - se * ca * p.z),
           ce * sa * p.x + se * p.y + ce * ca * p.z);
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
                      green = {75, 225, 168}, purple = {200, 125, 245};
static void arrow(const Scene *s, GRE_RenderTarget t, Vec p, Vec force,
                  GRErgb24 color) {
  if (norm(force) < 1e-8)
    return;
  Vec end = add(p, mul(unit(force), .22 + fmin(norm(force), 40) * .004));
  Vec a = project(s, p), b = project(s, end);
  double d = hypot(b.x - a.x, b.y - a.y);
  screen_line(t, a, b, color, 1);
  if (d < 1)
    return;
  double x = (b.x - a.x) / d, y = (b.y - a.y) / d;
  screen_line(t, b, V(b.x - 9 * x - 4 * y, b.y - 9 * y + 4 * x, b.z), color, 1);
  screen_line(t, b, V(b.x - 9 * x + 4 * y, b.y - 9 * y - 4 * x, b.z), color, 1);
}
static void moment(const Scene *s, GRE_RenderTarget t, Vec p, Vec m,
                   GRErgb24 color) {
  if (norm(m) < 1e-8)
    return;
  Vec axis = unit(m),
      u = unit(cross(axis, fabs(axis.y) < .8 ? V(0, 1, 0) : V(1, 0, 0))),
      v = cross(axis, u);
  Vec last = add(p, mul(u, .19));
  for (int i = 1; i <= 32; ++i) {
    double a = i * 1.6 * PI / 32;
    Vec now = add(p, mul(add(mul(u, cos(a)), mul(v, sin(a))), .19));
    line(s, t, last, now, color, 1);
    last = now;
  }
  Vec tangent = add(mul(u, -sin(1.6 * PI)), mul(v, cos(1.6 * PI)));
  Vec base = add(last, mul(tangent, -.06));
  line(s, t, last, add(base, mul(unit(add(last, mul(p, -1))), .025)), color, 1);
  line(s, t, last, add(base, mul(unit(add(last, mul(p, -1))), -.025)), color,
       1);
}
static Vec position(const Scene *s, size_t i) {
  return add(array(s->nodes[i].position),
             mul(array(s->result[i].displacement), s->amp));
}
static Vec sample(const Scene *s, size_t i, double q, Vec *rot) {
  const pme_statics3d_element *e = &s->elements[i];
  Vec a = array(s->nodes[e->node_a].position),
      b = array(s->nodes[e->node_b].position);
  Vec u = array(s->result[e->node_a].displacement),
      v = array(s->result[e->node_b].displacement);
  Vec ra = array(s->result[e->node_a].displacement + 3),
      rb = array(s->result[e->node_b].displacement + 3);
  if (e->kind != PME_STATICS_BEAM) {
    *rot = V(0, 0, 0);
    return add(add(mul(a, 1 - q), mul(b, q)),
               mul(add(mul(u, 1 - q), mul(v, q)), s->amp));
  }
  double axes[9], L = norm(add(b, mul(a, -1))), q2 = q * q, q3 = q2 * q;
  pme_statics3d_model m = {s->nodes, s->nc, s->elements, s->ec};
  (void)pme_statics3d_element_frame(&m, e, axes);
  Vec x = array(axes), y = array(axes + 3), z = array(axes + 6);
  double axial = (1 - q) * dot(u, x) + q * dot(v, x);
  double yy = (1 - 3 * q2 + 2 * q3) * dot(u, y) +
              (q - 2 * q2 + q3) * L * dot(ra, z) +
              (3 * q2 - 2 * q3) * dot(v, y) + (q3 - q2) * L * dot(rb, z);
  double zz = (1 - 3 * q2 + 2 * q3) * dot(u, z) -
              (q - 2 * q2 + q3) * L * dot(ra, y) +
              (3 * q2 - 2 * q3) * dot(v, z) - (q3 - q2) * L * dot(rb, y);
  double dy = (-6 * q + 6 * q2) * dot(u, y) / L +
              (1 - 4 * q + 3 * q2) * dot(ra, z) +
              (6 * q - 6 * q2) * dot(v, y) / L + (3 * q2 - 2 * q) * dot(rb, z);
  double dz = (-6 * q + 6 * q2) * dot(u, z) / L -
              (1 - 4 * q + 3 * q2) * dot(ra, y) +
              (6 * q - 6 * q2) * dot(v, z) / L - (3 * q2 - 2 * q) * dot(rb, y);
  *rot =
      mul(add(add(mul(x, (1 - q) * dot(ra, x) + q * dot(rb, x)), mul(y, -dz)),
              mul(z, dy)),
          s->amp);
  return add(add(mul(a, 1 - q), mul(b, q)),
             mul(add(add(mul(x, axial), mul(y, yy)), mul(z, zz)), s->amp));
}
static GRErgb24 color(double force, double maximum) {
  double q = maximum > 1e-12 ? fmin(fabs(force) / maximum, 1) : 0;
  return force >= 0 ? (GRErgb24){(uint8)(175 + 75 * q), (uint8)(195 - 95 * q),
                                 (uint8)(205 - 115 * q)}
                    : (GRErgb24){(uint8)(175 - 105 * q), (uint8)(195 - 50 * q),
                                 (uint8)(205 + 45 * q)};
}
static void draw(Scene *s, GRE_RenderTarget t) {
  for (size_t i = 0; i < (size_t)t->width * t->height; ++i) {
    t->data[i] = GRE_FramePixel_From_RGB24((GRErgb24){20, 28, 40});
    t->zbuff[i] = 1e20f;
  }
  double ground = s->scene == 2 ? 0 : -.4, maxforce = 0;
  for (int i = -4; i <= 6; ++i) {
    double x = i * .5;
    line(s, t, V(x, ground, -2), V(x, ground, 2), (GRErgb24){34, 48, 64}, 0);
    line(s, t, V(-2, ground, x), V(3, ground, x), (GRErgb24){34, 48, 64}, 0);
  }
  for (size_t i = 0; i < s->ec; ++i) {
    double v = s->scene == 2 ? fabs(s->force[i].axial_force)
               : s->scene == 3
                   ? fabs(s->force[i].end_force[9])
                   : hypot(s->force[i].end_force[4], s->force[i].end_force[5]);
    maxforce = fmax(maxforce, v);
  }
  for (size_t i = 0; i < s->ec; ++i) {
    const pme_statics3d_element *e = &s->elements[i];
    Vec r, last = sample(s, i, 0, &r);
    line(s, t, array(s->nodes[e->node_a].position),
         array(s->nodes[e->node_b].position), grey, 0);
    for (int k = 1; k <= 60; ++k) {
      double q = k / 60.;
      Vec p = sample(s, i, q, &r);
      double value = s->scene == 2 ? s->force[i].axial_force
                     : s->scene == 3
                         ? s->force[i].end_force[9]
                         : hypot(-(1 - q) * s->force[i].end_force[4] +
                                     q * s->force[i].end_force[10],
                                 -(1 - q) * s->force[i].end_force[5] +
                                     q * s->force[i].end_force[11]);
      line(s, t, last, p, color(value, maxforce), 2);
      last = p;
    }
    if (e->kind == PME_STATICS_BEAM) {
      for (int k = 0; k <= 8; ++k) {
        Vec p = sample(s, i, k / 8., &r), y = rotate(V(0, .09, 0), r),
            z = rotate(V(0, 0, .07), r);
        line(s, t, add(p, mul(y, -1)), add(p, y), green, 1);
        line(s, t, add(p, mul(z, -1)), add(p, z), purple, 1);
      }
    }
  }
  for (size_t i = 0; i < s->nc; ++i) {
    Vec p = position(s, i), scr = project(s, p);
    GRErgb24 c = i == s->selected ? gold : green;
    screen_line(t, scr, scr, c, s->scene == 2 ? 3 : 4);
    if (s->nodes[i].fixed) {
      Vec a = add(p, V(-.08, 0, -.08)), b = add(p, V(.08, 0, -.08)),
          c1 = add(p, V(.08, 0, .08)), d = add(p, V(-.08, 0, .08));
      line(s, t, a, b, grey, 1);
      line(s, t, b, c1, grey, 1);
      line(s, t, c1, d, grey, 1);
      line(s, t, d, a, grey, 1);
      if (s->nodes[i].fixed == 63) {
        line(s, t, add(p, V(0, -.1, 0)), add(p, V(0, .1, 0)), grey, 2);
      }
      arrow(s, t, p, array(s->result[i].reaction), green);
      moment(s, t, p, array(s->result[i].reaction + 3), green);
    }
    if (i == s->loaded) {
      arrow(s, t, p, array(s->nodes[i].load), gold);
      moment(s, t, p, array(s->nodes[i].load + 3), gold);
    }
  }
  /* Camera-oriented XYZ triad anchored in the bottom-left screen corner. */
  Vec origin = project(s, V(0, 0, 0));
  Vec axes[3] = {V(.28, 0, 0), V(0, .28, 0), V(0, 0, .28)};
  GRErgb24 colors[3] = {{245, 95, 95}, {85, 230, 145}, {90, 155, 255}};
  for (int i = 0; i < 3; ++i) {
    Vec tip = add(V(72, 435, -1e10), add(project(s, axes[i]), mul(origin, -1)));
    screen_line(t, V(72, 435, -1e10), tip, colors[i], 1);
  }
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
static int balanced(const Scene *s) {
  Vec f = V(0, 0, 0), m = V(0, 0, 0);
  for (size_t i = 0; i < s->nc; ++i) {
    Vec fi = add(array(s->nodes[i].load), array(s->result[i].reaction));
    f = add(f, fi);
    m = add(m, add(cross(array(s->nodes[i].position), fi),
                   add(array(s->nodes[i].load + 3),
                       array(s->result[i].reaction + 3))));
  }
  return norm(f) < 1e-7 && norm(m) < 1e-7;
}
int main(int argc, char **argv) {
  Scene s;
  YMGRE_DemoHost host;
  GRE_RenderTarget target = NULL;
  GYOBJ image, rows[14];
  int smoke = argc == 2 && !strcmp(argv[1], "--smoke-test"), start = 1, rc = 1;
  if (argc == 3 && !strcmp(argv[1], "--scene") && strlen(argv[2]) == 1 &&
      argv[2][0] >= '1' && argv[2][0] <= '3')
    start = argv[2][0] - '0';
  else if (argc != 1 && !smoke) {
    fprintf(stderr, "Usage: %s [--scene 1..3 | --smoke-test]\n", argv[0]);
    return 1;
  }
  reset(&s, start);
  if (!solve(&s))
    return 1;
  YMGRE_DemoHost_DefaultScale(1);
  if (!YMGRE_DemoHost_Init(&host, 1200, 720, 40)) {
    fprintf(stderr, "GRE: %s\n", YMGRE_DemoHost_LastError());
    return 1;
  }
  YMGRE_DemoHost_SetTitle(&host, "YMPME | 3D statics | 3 examples");
  target = YMGRE_Creat_RenderTarget(840, 500);
  if (!target)
    goto done;
  image = YMGRE_DemoHost_AddTarget(&host, target, 10, 60, 840, 500);
  if (!image ||
      !label(host.context->root, 15, 15, 1170,
             "YMPME / 3D STATICS    3 examples    Press 1-3 to switch"))
    goto done;
  for (int i = 0; i < 14; ++i) {
    rows[i] = label(host.context->root, 865, 65 + 35 * i, 325, "");
    if (!rows[i])
      goto done;
  }
  YMGUI_Label_SetTextColor(rows[0], GY_ARGB(255, 255, 208, 88));
  const char *notes[8] = {"1: biaxial beam  2: space truss  3: torsion",
                          "Arrows: Fx/Fy  Q/E: Fz  G/T: Mx (beam)",
                          "A/D: orbit  W/S: elevation  +/-: zoom",
                          "[/]: deformation scale  C: reset camera",
                          "TAB: node  R: reset case  ESC: exit",
                          "Yellow: applied load  Green: reaction",
                          "XYZ axes: red / green / blue",
                          "Section crosses: orientation (schematic)"};
  for (int i = 0; i < 8; ++i)
    if (!label(host.context->root, 15 + 590 * (i % 2), 575 + 33 * (i / 2), 580,
               notes[i]))
      goto done;
  if (!YMGRE_DemoHost_BindKeys(&host, keys, sizeof keys / sizeof keys[0]))
    goto done;
  unsigned covered = 0;
  uint64_t original = 0;
  Vec anchor = V(0, 0, 0);
  double saved_yaw = 0;
  for (unsigned frame = 0;; ++frame) {
    unsigned phase = frame % 12;
    char text[96];
    if (smoke) {
      int key = frame == 36  ? YMGRE_KEY_ESCAPE
                : phase == 0 ? '1' + (int)(frame / 12)
                : phase == 2 ? 'e'
                : phase == 4 ? 'd'
                : phase == 6 ? YMGRE_KEY_TAB
                : phase == 8 ? 'r'
                             : 0;
      if (key && (!YMGRE_DemoHost_InjectKey(&host, key, 1) ||
                  !YMGRE_DemoHost_InjectKey(&host, key, 0)))
        goto done;
    }
    unsigned a = YMGRE_DemoHost_TakeActions(&host);
    int selection = YMGRE_DemoHost_TakeValue(&host);
    if (a & QUIT) {
      rc = smoke && covered != 7 ? 1 : 0;
      break;
    }
    if (selection)
      reset(&s, selection);
    if (a & RESET)
      reset(&s, s.scene);
    double *load = s.nodes[s.loaded].load;
    if (a & FXP)
      load[0] += 5;
    if (a & FXM)
      load[0] -= 5;
    if (a & FYP)
      load[1] += 5;
    if (a & FYM)
      load[1] -= 5;
    if (a & FZP)
      load[2] += 5;
    if (a & FZM)
      load[2] -= 5;
    if (s.scene != 2) {
      if (a & MXP)
        load[3] += 1;
      if (a & MXM)
        load[3] -= 1;
    }
    for (int i = 0; i < 4; ++i)
      load[i] = fmax(i == 3 ? -6 : -40, fmin(i == 3 ? 6 : 40, load[i]));
    if (a & YAWP)
      s.yaw += .12;
    if (a & YAWM)
      s.yaw -= .12;
    if (a & PITCHP)
      s.pitch += .1;
    if (a & PITCHM)
      s.pitch -= .1;
    s.pitch = fmax(-1.3, fmin(1.3, s.pitch));
    if (a & ZOOMP)
      s.zoom *= 1.1;
    if (a & ZOOMM)
      s.zoom /= 1.1;
    s.zoom = fmax(50, fmin(400, s.zoom));
    if (a & AMPP)
      s.amp *= 1.25;
    if (a & AMPM)
      s.amp /= 1.25;
    s.amp = fmax(1, fmin(3000, s.amp));
    if (a & CAMERA)
      camera_reset(&s);
    if (a & NEXT)
      s.selected = (s.selected + 1) % s.nc;
    if (!solve(&s))
      goto done;
    draw(&s, target);
    YMGUI_Label_SetText(rows[0], s.scene == 1   ? "1/3  BIAXIAL CANTILEVER"
                                 : s.scene == 2 ? "2/3  SPACE TRUSS"
                                                : "3/3  TORSION / BEAM");
    snprintf(text, sizeof text, "F: %+.0f %+.0f %+.0f N", load[0], load[1],
             load[2]);
    YMGUI_Label_SetText(rows[1], text);
    snprintf(text, sizeof text, "Mx: %+.1f N m / amp x%.0f", load[3], s.amp);
    YMGUI_Label_SetText(rows[2], text);
    snprintf(text, sizeof text, "Node %zu / load at %zu", s.selected, s.loaded);
    YMGUI_Label_SetText(rows[3], text);
    const double *u = s.result[s.selected].displacement,
                 *r = s.result[s.selected].reaction;
    for (int i = 0; i < 3; ++i) {
      snprintf(text, sizeof text, "u%c = %+.5f mm", 'x' + i, u[i] * 1000);
      YMGUI_Label_SetText(rows[4 + i], text);
    }
    for (int i = 0; i < 3; ++i) {
      snprintf(text, sizeof text, "r%c = %+.6f rad", 'x' + i, u[3 + i]);
      YMGUI_Label_SetText(rows[7 + i], text);
    }
    snprintf(text, sizeof text, "R: %+.2f %+.2f %+.2f N", r[0], r[1], r[2]);
    YMGUI_Label_SetText(rows[10], text);
    snprintf(text, sizeof text, "M: %+.2f %+.2f %+.2f Nm", r[3], r[4], r[5]);
    YMGUI_Label_SetText(rows[11], text);
    YMGUI_Label_SetText(rows[12],
                        s.scene == 2   ? "Color: N (+tension / -compression)"
                        : s.scene == 3 ? "Color: torsion (+red / -blue)"
                                       : "Color: bending magnitude |M|");
    YMGUI_Label_SetText(rows[13], "View locked until camera keys");
    if (smoke) {
      if (!balanced(&s))
        goto done;
      if (phase == 1) {
        if (s.scene != (int)(frame / 12) + 1)
          goto done;
        original = hash(target);
        anchor = project(&s, array(s.nodes[0].position));
        saved_yaw = s.yaw;
      }
      if (phase == 3) {
        Vec now = project(&s, position(&s, 0));
        if (norm(add(now, mul(anchor, -1))) > 1e-9 || hash(target) == original)
          goto done;
      }
      if (phase == 5 && s.yaw == saved_yaw)
        goto done;
      if (phase == 7 && s.selected != (s.loaded + 1) % s.nc)
        goto done;
      if (phase == 9) {
        if (s.selected != s.loaded || s.yaw != saved_yaw)
          goto done;
        covered |= 1u << (s.scene - 1);
      }
      if (frame > 40)
        goto done;
    }
    YMGUI_Obj_Invalidate(image);
    if (!YMGRE_DemoHost_Step(&host, smoke ? 0 : 30)) {
      rc = smoke ? 1 : 0;
      break;
    }
  }
  if (smoke)
    printf("%s 3D viewer: cases, loads, fixed screen anchor, camera, "
           "selection, reset (%u)\n",
           rc ? "FAIL" : "PASS", covered);
done:
  YMGRE_DemoHost_Destroy(&host);
  if (target)
    YMGRE_Free_RenderTarget(target);
  return rc;
}
