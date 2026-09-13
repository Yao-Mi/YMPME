#include "YMGUI_Invalidate.h"
#include "YMGUI_Label.h"
#include "demo_host.h"
#include "scene.h"
#include <stdio.h>
#include <string.h>
enum {
  PAUSE = 1,
  STEP = 2,
  RESET = 4,
  MOTOR = 8,
  FRICTION = 16,
  CAMERA = 32,
  WIRE = 64,
  MARKERS = 128,
  QUIT = 256,
  REVERSE = 512,
  STOP = 1024
};
enum { FASTER = 1, SLOWER = 2, TIGHTER = 4, LOOSER = 8 };
static const YMGRE_DemoKeyBinding keys[] = {{YMGRE_KEY_SPACE, 0, PAUSE, 0},
                                            {'n', 0, STEP, 0},
                                            {'r', 0, RESET, 0},
                                            {'m', 0, MOTOR, 0},
                                            {'f', 0, FRICTION, 0},
                                            {'c', 0, CAMERA, 0},
                                            {'v', 0, WIRE, 0},
                                            {'h', 0, MARKERS, 0},
                                            {YMGRE_KEY_ESCAPE, 0, QUIT, 0},
                                            {'d', 0, REVERSE, 0},
                                            {'s', 0, STOP, 0},
                                            {YMGRE_KEY_UP, FASTER, 0, 0},
                                            {YMGRE_KEY_DOWN, SLOWER, 0, 0},
                                            {']', TIGHTER, 0, 0},
                                            {'[', LOOSER, 0, 0}};
static GYOBJ label(GYOBJ parent, int y) {
  GYOBJ l = YMGUI_Creat_Label_Creat(parent, 16, y, 1068, 25);
  if (l)
    YMGUI_Label_SetTextColor(l, GY_ARGB(255, 225, 235, 244));
  return l;
}
static int tap(YMGRE_DemoHost *h, int k) {
  return YMGRE_DemoHost_InjectKey(h, k, 1) && YMGRE_DemoHost_InjectKey(h, k, 0);
}
static int step(track_lab *s, unsigned held) {
  s->speed = YMPME_Clamp(
      s->speed + (((held & FASTER) != 0) - ((held & SLOWER) != 0)) * TRACK_DT,
      -3, 3);
  s->preload = YMPME_Clamp(
      s->preload +
          (((held & TIGHTER) != 0) - ((held & LOOSER) != 0)) * 30 * TRACK_DT,
      0, 100);
  return Track_Step(s);
}
int main(int argc, char **argv) {
  static track_lab state;
  static track_scene scene;
  YMGRE_DemoHost host;
  GYOBJ rows[7], target;
  int ready = 0, result = 1, paused = 0, view = 0, wire = 0, markers = 1;
  int smoke = argc == 2 && !strcmp(argv[1], "--smoke-test");
  unsigned long saved = 0;
  double accumulator = 0, previous, stepMs = 0;
  if (argc > 1 && !smoke) {
    fprintf(stderr, "Usage: %s [--smoke-test]\n", argv[0]);
    return 1;
  }
  if (!Track_Init(&state) || !TrackScene_Init(&scene, &state))
    goto done;
  YMGRE_DemoHost_DefaultScale(1);
  if (!YMGRE_DemoHost_Init(&host, 1100, 780, 40)) {
    fprintf(stderr, "GRE: %s\n", YMGRE_DemoHost_LastError());
    goto done;
  }
  ready = 1;
  YMGRE_DemoHost_SetTitle(&host,
                          "YMPME | Physical track bench | DYNAMICS + GRE");
  target = YMGRE_DemoHost_AddTarget(
      &host, YMGRE_Camera_GetRenderTarget(scene.camera), 0, 0, 1100, 560);
  if (!target)
    goto done;
  for (int i = 0; i < 7; ++i) {
    rows[i] = label(host.context->root, 568 + i * 29);
    if (!rows[i])
      goto done;
  }
  if (!YMGRE_DemoHost_BindKeys(&host, keys, sizeof keys / sizeof keys[0]))
    goto done;
  previous = YMGRE_DemoHost_Time();
  for (int frame = 0;; ++frame) {
    double now = YMGRE_DemoHost_Time(),
           elapsed = smoke ? 1. / 60 : now - previous;
    previous = now;
    if (smoke) {
      if (frame == 5) {
        saved = state.steps;
        if (!tap(&host, YMGRE_KEY_SPACE))
          goto done;
      }
      if (frame == 8) {
        if (state.steps != saved || !tap(&host, 'n'))
          goto done;
      }
      if (frame == 9 && state.steps != saved + 1)
        goto done;
      if (frame == 10 && !tap(&host, YMGRE_KEY_SPACE))
        goto done;
      if (frame == 15 && !tap(&host, 'c'))
        goto done;
      if (frame == 20 && !tap(&host, 'v'))
        goto done;
      if (frame == 25 && !tap(&host, 'h'))
        goto done;
      if (frame == 150) {
        if (state.beltSpeed < .15f || state.anchorError > .01f || view != 1 ||
            !wire || markers)
          goto done;
        if (!tap(&host, 'f') || !tap(&host, 'm') || !tap(&host, 'd'))
          goto done;
      }
      if (frame == 152) {
        if (state.friction != 0 || state.motor || state.speed >= 0)
          goto done;
        if (!tap(&host, 'r'))
          goto done;
      }
      if (frame == 154) {
        if (state.steps > 32 || state.friction <= 0 || !state.motor ||
            state.speed <= 0)
          goto done;
        if (!tap(&host, YMGRE_KEY_ESCAPE))
          goto done;
      }
    }
    unsigned actions = YMGRE_DemoHost_TakeActions(&host),
             held = YMGRE_DemoHost_HeldKeys(&host);
    if (actions & QUIT) {
      result = 0;
      break;
    }
    if (actions & RESET) {
      if (!Track_Init(&state))
        goto done;
      paused = 0;
      accumulator = 0;
      YMGRE_DemoHost_ClearInput(&host);
      held = 0;
    }
    if (actions & PAUSE) {
      paused = !paused;
      accumulator = 0;
    }
    if (actions & MOTOR)
      state.motor = !state.motor;
    if (actions & FRICTION)
      state.friction = state.friction > 0 ? 0 : 1.1f;
    if (actions & REVERSE)
      state.speed = -state.speed;
    if (actions & STOP)
      state.speed = 0;
    if (actions & CAMERA)
      view = !view;
    if (actions & WIRE)
      wire = !wire;
    if (actions & MARKERS)
      markers = !markers;
    double started = YMGRE_DemoHost_Time();
    unsigned long before = state.steps;
    if (paused && (actions & STEP) && !step(&state, held))
      goto done;
    if (!paused) {
      /* Bound catch-up work. Under overload simulation slows instead of
       * spiralling. */
      accumulator = fmin(accumulator + fmin(elapsed, .05), 16. * TRACK_DT);
      while (accumulator >= TRACK_DT) {
        if (!step(&state, held))
          goto done;
        accumulator -= TRACK_DT;
      }
    }
    if (state.steps > before) {
      double ms =
          1000 * (YMGRE_DemoHost_Time() - started) / (state.steps - before);
      stepMs = stepMs ? stepMs * .9 + ms * .1 : ms;
    }
    TrackScene_Render(&scene, &state, view, wire, markers);
    YMGUI_Obj_Invalidate(target);
    char text[192];
    snprintf(text, sizeof text,
             "PHYSICAL TRACK | %s | 40 links / 43 joints | 480 Hz, 64 "
             "iterations | t %.2f s",
             paused ? "PAUSED" : "RUN", state.world.time);
    YMGUI_Label_SetText(rows[0], text);
    snprintf(text, sizeof text,
             "Drive target %+.2f / actual %+.2f rad/s | belt %.3f m/s | motor "
             "%s | mu %.2f",
             state.speed, state.bodies[40].angularVelocity.z, state.beltSpeed,
             state.motor ? "ON" : "OFF", state.friction);
    YMGUI_Label_SetText(rows[1], text);
    snprintf(text, sizeof text,
             "Motor torque %+.3f Nm | preload %.1f N | spring %.1f N | slider "
             "%+.2f mm",
             state.joints[40].feedback.motorEffort, state.preload,
             state.tension,
             1000 * (state.bodies[42].position.x - state.halfSpan));
    YMGUI_Label_SetText(rows[2], text);
    snprintf(text, sizeof text,
             "Hinge error %.3f mm | contact depth %.3f mm | contacts %zu | "
             "physics %.3f ms/step (%.1f ms/frame @60)",
             state.anchorError * 1000, state.penetration * 1000,
             state.world.contactCount, stepMs, stepMs * 8);
    YMGUI_Label_SetText(rows[3], text);
    YMGUI_Label_SetText(rows[4], "UP/DOWN Speed | D Reverse | S Stop | [ / ] "
                                 "Spring preload | M Motor | F Friction");
    YMGUI_Label_SetText(rows[5],
                        "SPACE Pause | N Single step | R Reset | C Camera | V "
                        "Wireframe | H Contacts/hinges | ESC Exit");
    YMGUI_Label_SetText(rows[6],
                        "Friction drive; no tooth engagement. Green: loaded "
                        "links/contact normals. Red: one tracked link.");
    if (!YMGRE_DemoHost_Step(&host, smoke ? 0 : 60)) {
      result = smoke ? 1 : 0;
      break;
    }
  }
done:
  if (ready)
    YMGRE_DemoHost_Destroy(&host);
  TrackScene_Destroy(&scene);
  if (result)
    fprintf(stderr, "Track bench failed: dynamics status %d at %.3f s\n",
            state.status, state.world.time);
  if (smoke)
    printf("TRACK_WINDOW %s\n", result ? "FAIL" : "PASS");
  return result;
}
