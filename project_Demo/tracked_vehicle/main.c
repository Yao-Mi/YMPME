#include "YMGUI_Invalidate.h"
#include "YMGUI_Label.h"
#include "demo_host.h"
#include "scene.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
enum {
    PAUSE = 1,
    STEP = 2,
    RESET = 4,
    AUTO = 8,
    FRICTION = 16,
    MOTOR = 32,
    CAMERA = 64,
    WIRE = 128,
    FORCES = 256,
    QUIT = 512
};
enum { FORWARD = 1, BACK = 2, LEFT = 4, RIGHT = 8, BRAKE = 16 };
static const YMGRE_DemoKeyBinding keys[] = {{YMGRE_KEY_UP, FORWARD, 0, 0},
                                            {'w', FORWARD, 0, 0},
                                            {YMGRE_KEY_DOWN, BACK, 0, 0},
                                            {'s', BACK, 0, 0},
                                            {YMGRE_KEY_LEFT, LEFT, 0, 0},
                                            {'a', LEFT, 0, 0},
                                            {YMGRE_KEY_RIGHT, RIGHT, 0, 0},
                                            {'d', RIGHT, 0, 0},
                                            {'b', BRAKE, 0, 0},
                                            {YMGRE_KEY_SPACE, 0, PAUSE, 0},
                                            {'n', 0, STEP, 0},
                                            {'r', 0, RESET, 0},
                                            {'t', 0, AUTO, 0},
                                            {'f', 0, FRICTION, 0},
                                            {'m', 0, MOTOR, 0},
                                            {'c', 0, CAMERA, 0},
                                            {'v', 0, WIRE, 0},
                                            {'h', 0, FORCES, 0},
                                            {YMGRE_KEY_ESCAPE, 0, QUIT, 0},
                                            {'1', 0, 0, 1},
                                            {'2', 0, 0, 2},
                                            {'3', 0, 0, 3},
                                            {'4', 0, 0, 4}};
static GYOBJ label(GYOBJ root, int y, const char *text) {
    GYOBJ o = YMGUI_Creat_Label_Creat(root, 1018, y, 370, 25);
    if (o) {
        YMGUI_Label_SetText(o, text);
        YMGUI_Label_SetTextColor(o, GY_ARGB(255, 227, 236, 243));
    }
    return o;
}
static int tap(YMGRE_DemoHost *h, int key) {
    return YMGRE_DemoHost_InjectKey(h, key, 1) && YMGRE_DemoHost_InjectKey(h, key, 0);
}
static int advance(tracked_vehicle *s, unsigned held, int automatic) {
    float throttle = (float)(((held & FORWARD) != 0) - ((held & BACK) != 0));
    float steer = (float)(((held & RIGHT) != 0) - ((held & LEFT) != 0));
    if (automatic && !(held & (FORWARD | BACK | LEFT | RIGHT | BRAKE)))
        throttle = s->scenario >= 2 ? .6f : 1;
    return TV_Step(s, throttle, steer, (held & BRAKE) != 0);
}
static uint64_t hash(GRE_RenderTarget t) {
    uint64_t h = UINT64_C(14695981039346656037);
    const unsigned char *p = (const unsigned char *)t->data;
    for (size_t i = 0; i < (size_t)t->width * t->height * sizeof(*t->data); ++i)
        h = (h ^ p[i]) * UINT64_C(1099511628211);
    return h;
}
/* Optional smoke-test evidence, not required to run the window. */
static int capture(GRE_RenderTarget t, const char *suffix) {
    const char *path = getenv("YMPME_TRACKED_CAPTURE");
    if (!path || !*path)
        return 1;
    char name[4096];
    int size = snprintf(name, sizeof(name), "%s%s", path, suffix);
    if (size < 0 || (size_t)size >= sizeof(name))
        return 0;
    FILE *f = fopen(name, "wb");
    if (!f)
        return 0;
    fprintf(f, "P6\n%u %u\n255\n", t->width, t->height);
    for (size_t i = 0; i < (size_t)t->width * t->height; ++i) {
        GRErgb24 c = GRE_FramePixel_To_RGB24(t->data[i]);
        unsigned char pixel[3] = {c.R, c.G, c.B};
        if (fwrite(pixel, 1, 3, f) != 3) {
            fclose(f);
            return 0;
        }
    }
    return fclose(f) == 0;
}
static int renderedLinksMatch(const tv_scene *scene, const tracked_vehicle *v) {
    int links = 0;
    for (int i = 0; i < scene->count; ++i) {
        const tv_part *part = &scene->parts[i];
        const pme_collider *c = &v->colliders[part->collider];
        if (TV_LinkSide(c->body) < 0)
            continue;
        const pme_rigid_body *b = &v->bodies[c->body];
        pme_rotation r = YMPME_Quaternion_Rotation(b->orientation);
        for (int j = 0; j < part->object->pointNum; ++j) {
            gre_fvector4d p = part->object->pointList[j].pos;
            pme_vec3 local = YMPME_Rotation_InverseApply(
                r, YMPME_VecSub(YMPME_Vec3(p.x / 100, p.y / 100, p.z / 100), b->position));
            if (fabsf(fabsf(local.x) - c->halfSize.x) > .00005f ||
                fabsf(fabsf(local.y) - c->halfSize.y) > .00005f ||
                fabsf(fabsf(local.z) - c->halfSize.z) > .00005f)
                return 0;
        }
        ++links;
    }
    return links == TV_LINKS;
}
int main(int argc, char **argv) {
    static tracked_vehicle state;
    static tv_scene scene;
    YMGRE_DemoHost host;
    int smoke = argc == 2 && !strcmp(argv[1], "--smoke-test");
    int scenario = argc == 2 && !strcmp(argv[1], "--wall")    ? 1
                   : argc == 2 && !strcmp(argv[1], "--bumps") ? 2
                   : argc == 2 && !strcmp(argv[1], "--crest") ? 3
                                                              : 0;
    if (argc > 1 && !smoke && !scenario) {
        fprintf(stderr, "Usage: %s [--wall|--bumps|--crest|--smoke-test]\n", argv[0]);
        return 1;
    }
    int frame = -1;
    int ready = 0, result = 1, paused = 0, automatic = scenario == 1, view = 0, wire = 0,
        forces = 1, verified = 0;
    unsigned long saved = 0;
    uint64_t first = 0;
    double previous, accumulator = 0, speedRatio = 1, frameMs = 1000. / 60;
    if (!TV_Init(&state, scenario) || !TVScene_Init(&scene, &state))
        goto done;
    YMGRE_DemoHost_DefaultScale(1);
    if (!YMGRE_DemoHost_Init(&host, 1400, 700, 40)) {
        fprintf(stderr, "GRE: %s\n", YMGRE_DemoHost_LastError());
        goto done;
    }
    ready = 1;
    char title[128], heading[80];
    snprintf(title, sizeof(title), "YMPME + GRE | %d rigid track links | wheel contact drive", TV_LINKS);
    snprintf(heading, sizeof(heading), "PHYSICAL TRACKS / %d RIGID LINKS", TV_LINKS);
    YMGRE_DemoHost_SetTitle(&host, title);
    GYOBJ image = YMGRE_DemoHost_AddTarget(&host, YMGRE_Camera_GetRenderTarget(scene.camera), 0, 0,
                                           1000, 650);
    if (!image || !label(host.context->root, 8, heading))
        goto done;
    GYOBJ rows[15];
    for (int i = 0; i < 15; ++i) {
        rows[i] = label(host.context->root, 40 + 24 * i, "");
        if (!rows[i])
            goto done;
    }
    const char *help[] = {
        "WASD / arrows : drive + steer",       "B brake | T auto push | R reset",
        "1 Free | 2 Wall | 3 Bumps | 4 Crest", "F ground friction | M drive motors",
        "C camera | V wire | H forces/joints", "SPACE pause | N physics step | ESC",
        "Orange: one actual link per loop",    "Wheels + flanges use collision mesh",
        "240 Hz / max 8 steps per frame"};
    for (int i = 0; i < 9; ++i)
        if (!label(host.context->root, 430 + 28 * i, help[i]))
            goto done;
    if (!YMGRE_DemoHost_BindKeys(&host, keys, sizeof(keys) / sizeof(keys[0])))
        goto done;
    previous = YMGRE_DemoHost_Time();
    for (frame = 0;; ++frame) {
        double now = YMGRE_DemoHost_Time(), elapsed = smoke ? 1. / 60 : now - previous;
        previous = now;
        if (smoke) {
            if (frame == 2 && !YMGRE_DemoHost_InjectKey(&host, 'w', 1))
                goto done;
            if (frame == 10) {
                saved = state.steps;
                if (!tap(&host, YMGRE_KEY_SPACE))
                    goto done;
            }
            if (frame == 13 && (state.steps != saved || !tap(&host, 'n')))
                goto done;
            if (frame == 14 && state.steps != saved + 1)
                goto done;
            if (frame == 16 && !tap(&host, YMGRE_KEY_SPACE))
                goto done;
            if (frame == 100) {
                /* The drive target preserves linear speed when wheel size changes. */
                if (state.bodies[0].position.z + 3 < .24f ||
                    state.beltSpeed[0] < .15f ||
                    first == hash(YMGRE_Camera_GetRenderTarget(scene.camera)))
                    goto done;
                if (!capture(YMGRE_Camera_GetRenderTarget(scene.camera), ".start.ppm") ||
                    !YMGRE_DemoHost_InjectKey(&host, 'w', 0))
                    goto done;
                verified = 1;
            }
            if (frame == 102 && (!tap(&host, 'c') || !tap(&host, 'v') || !tap(&host, 'h') ||
                                 !tap(&host, 'f') || !tap(&host, 'm')))
                goto done;
            if (frame == 103 &&
                (view != 1 || !wire || forces || state.friction != 0 || state.motor))
                goto done;
            if (frame == 110 && !tap(&host, 'r'))
                goto done;
            if (frame == 111 && (!state.motor || state.friction == 0))
                goto done;
            if (frame == 115 && !tap(&host, '2'))
                goto done;
            if (frame == 116 && state.scenario != 1)
                goto done;
            if (frame == 120 && !tap(&host, '4'))
                goto done;
            if (frame == 121 && (state.scenario != 3 || !tap(&host, 'c')))
                goto done;
            if (frame == 122 && !tap(&host, 'c'))
                goto done;
            if (frame == 123 && !tap(&host, 'v'))
                goto done;
            if (frame == 190) {
                if (state.anchorError > .01f || state.penetration > .015f ||
                    !capture(YMGRE_Camera_GetRenderTarget(scene.camera), ".crest.ppm"))
                    goto done;
            }
            if (frame == 195 && !tap(&host, 't'))
                goto done;
            if (frame == 205 && (!automatic || !tap(&host, 't')))
                goto done;
            if (frame == 210 && !tap(&host, 'r'))
                goto done;
            if (frame == 215 && !tap(&host, YMGRE_KEY_ESCAPE))
                goto done;
        }
        unsigned actions = YMGRE_DemoHost_TakeActions(&host), held = YMGRE_DemoHost_HeldKeys(&host);
        int value = YMGRE_DemoHost_TakeValue(&host);
        if (actions & QUIT) {
            result = smoke ? !verified : 0;
            break;
        }
        if ((actions & RESET) || value) {
            if (!TV_Init(&state, value ? value - 1 : state.scenario))
                goto done;
            accumulator = 0;
            paused = 0;
            YMGRE_DemoHost_ClearInput(&host);
            held = 0;
        }
        if (actions & PAUSE) {
            paused = !paused;
            accumulator = 0;
        }
        if (actions & AUTO)
            automatic = !automatic;
        if (actions & FRICTION)
            state.friction = state.friction > 0 ? 0 : .3f;
        if (actions & MOTOR)
            state.motor = !state.motor;
        if (actions & CAMERA)
            view = (view + 1) % 3;
        if (actions & WIRE)
            wire = !wire;
        if (actions & FORCES)
            forces = !forces;
        double simBefore = state.world.time;
        double begin = YMGRE_DemoHost_Time();
        if (paused && (actions & STEP) && !advance(&state, held, automatic))
            goto done;
        if (!paused) {
            accumulator = fmin(accumulator + fmin(elapsed, .05), 8 * (double)TV_DT);
            for (int sub = 0; sub < 8 && accumulator >= TV_DT; ++sub) {
                if (!advance(&state, held, automatic))
                    goto done;
                accumulator -= TV_DT;
            }
        }
        double physicsMs = (YMGRE_DemoHost_Time() - begin) * 1000;
        if (elapsed > 0) {
            frameMs += .1 * (elapsed * 1000 - frameMs);
            if (!paused)
                speedRatio += .1 * ((state.world.time - simBefore) / elapsed - speedRatio);
        }
        TVScene_Render(&scene, &state, view, wire, forces);
        if (smoke && !renderedLinksMatch(&scene, &state))
            goto done;
        YMGUI_Obj_Invalidate(image);
        if (!frame)
            first = hash(YMGRE_Camera_GetRenderTarget(scene.camera));
        pme_rigid_body *b = &state.bodies[0];
        pme_rotation r = YMPME_Quaternion_Rotation(b->orientation);
        pme_vec3 omega = YMPME_Rotation_InverseApply(r, b->angularVelocity);
        pme_vec3 momentum = YMPME_Rotation_Apply(
            r, YMPME_Vec3(b->inertia.x * omega.x, b->inertia.y * omega.y, b->inertia.z * omega.z));
        char text[100];
#define ROW(i, ...)                                                                                \
    do {                                                                                           \
        snprintf(text, sizeof(text), __VA_ARGS__);                                                 \
        YMGUI_Label_SetText(rows[i], text);                                                        \
    } while (0)
        ROW(0, "%s | %s | %s",
            state.scenario == 1   ? "WALL"
            : state.scenario == 2 ? "BUMPS"
            : state.scenario == 3 ? "CREST"
                                  : "FREE",
            automatic ? "AUTO" : "MANUAL", paused ? "PAUSED" : "RUN");
        ROW(1, "%.0f fps | sim %.2fx | physics %.1f ms", 1000 / frameMs, speedRatio, physicsMs);
        ROW(2, "v %.2f %.2f %.2f m/s", b->velocity.x, b->velocity.y, b->velocity.z);
        ROW(3, "yaw %.1f deg | rate %.3f rad/s", TV_Yaw(&state) * 57.29578f, b->angularVelocity.y);
        ROW(4, "hull angular momentum Y %+.2f", momentum.y);
        ROW(5, "drive shafts %+.2f / %+.2f rad/s", state.wheelSpeed[0], state.wheelSpeed[1]);
        ROW(6, "motor torque %+.2f / %+.2f Nm", state.motorTorque[0], state.motorTorque[1]);
        ROW(7, "link mean speed %.2f / %.2f m/s", state.beltSpeed[0], state.beltSpeed[1]);
        ROW(8, "ground %.1f N / weight %.1f N", state.support, state.mass * 9.81f);
        ROW(9, "wall Fx %+.1f | Fz %+.1f N", state.wallLoad.force.x, state.wallLoad.force.z);
        ROW(10, "wall torque Y %+.2f Nm", state.wallLoad.torque.y);
        ROW(11, "tension spring %.1f / %.1f N", state.tension[0], state.tension[1]);
        ROW(12, "mu %.2f | motor %s | contacts %zu", state.friction, state.motor ? "ON" : "OFF",
            state.world.contactCount);
        ROW(13, "overlap %.2f | joint %.2f mm", state.penetration * 1000, state.anchorError * 1000);
        ROW(14, "track axial offset %.2f mm", state.lateralError * 1000);
#undef ROW
        /* DemoHost waits a full 1000/fps ms; budget only the time left after our work. */
        double spare = 1. / 60 - (YMGRE_DemoHost_Time() - now);
        int waitFps = !smoke && spare >= .001 ? (int)ceil(1. / spare) : 0;
        if (!YMGRE_DemoHost_Step(&host, waitFps)) {
            result = smoke ? 1 : 0;
            break;
        }
    }
done:
    if (ready)
        YMGRE_DemoHost_Destroy(&host);
    TVScene_Destroy(&scene);
    if (smoke)
        printf("TRACKED_WINDOW %s: real links, GRE inputs, reset, crest, render\n",
               result ? "FAIL" : "PASS");
    if (result)
        fprintf(stderr, "Tracked vehicle failed (frame %d, physics status %d, z %.3f, belt %.3f)\n",
                frame, state.status, state.bodies[0].position.z, state.beltSpeed[0]);
    return result;
}
