#include "YMGUI_Invalidate.h"
#include "YMGUI_Label.h"
#include "demo_host.h"
#include "scene.h"
#include <stdio.h>
#include <string.h>
enum
{
    PAUSE = 1,
    STEP = 2,
    RESET = 4,
    BOUNCE = 8,
    WIRE = 16,
    QUIT = 32,
    PLANE = 64,
    BOX = 128,
    MESH = 256
};
static const YMGRE_DemoKeyBinding keys[] = {{YMGRE_KEY_SPACE, 0, PAUSE, 0},
                                            {'n', 0, STEP, 0},
                                            {'r', 0, RESET, 0},
                                            {'b', 0, BOUNCE, 0},
                                            {'v', 0, WIRE, 0},
                                            {YMGRE_KEY_ESCAPE, 0, QUIT, 0},
                                            {'1', 0, PLANE, 0},
                                            {'2', 0, BOX, 0},
                                            {'3', 0, MESH, 0}};
static int tap(YMGRE_DemoHost *h, int k)
{
    return YMGRE_DemoHost_InjectKey(h, k, 1) && YMGRE_DemoHost_InjectKey(h, k, 0);
}
int main(int argc, char **argv)
{
    static ccd_lab state;
    static ccd_scene scene;
    YMGRE_DemoHost host;
    GYOBJ rows[6], target;
    int smoke = argc == 2 && !strcmp(argv[1], "--smoke-test"), ready = 0, result = 1, paused = 0,
        wire = 0;
    double accumulator = 0, previous;
    if (argc > 1 && !smoke)
    {
        fprintf(stderr, "Usage: %s [--smoke-test]\n", argv[0]);
        return 1;
    }
    if (!CCDLab_Init(&state, 1, 0) || !CCDScene_Init(&scene))
        goto done;
    YMGRE_DemoHost_DefaultScale(1);
    if (!YMGRE_DemoHost_Init(&host, 1000, 780, 40))
        goto done;
    ready = 1;
    YMGRE_DemoHost_SetTitle(&host, "YMPME + GRE | CCD thin wall comparison");
    target = YMGRE_DemoHost_AddTarget(&host, YMGRE_Camera_GetRenderTarget(scene.camera), 0, 0, 1000,
                                      580);
    if (!target)
        goto done;
    for (int i = 0; i < 6; ++i)
    {
        rows[i] = YMGUI_Creat_Label_Creat(host.context->root, 16, 585 + i * 30, 968, 28);
        if (!rows[i])
            goto done;
        YMGUI_Label_SetTextColor(rows[i], GY_ARGB(255, 224, 235, 244));
    }
    if (!YMGRE_DemoHost_BindKeys(&host, keys, sizeof keys / sizeof keys[0]))
        goto done;
    previous = YMGRE_DemoHost_Time();
    for (int frame = 0;; ++frame)
    {
        double now = YMGRE_DemoHost_Time(), elapsed = smoke ? 1. / 60 : now - previous;
        previous = now;
        if (smoke)
        {
            if (frame == 32)
            {
                if (state.steps != 1 || !state.lanes[1].hits ||
                    state.lanes[0].bodies[1].position.x < .9f ||
                    state.lanes[1].bodies[1].position.x > 0 || !tap(&host, YMGRE_KEY_SPACE))
                    goto done;
            }
            if (frame == 35 && (state.steps != 1 || !tap(&host, 'n')))
                goto done;
            if (frame == 37 && (state.steps != 2 || !tap(&host, 'b')))
                goto done;
            if (frame == 39 && (!state.bounce || state.steps != 0 || !tap(&host, '3')))
                goto done;
            if (frame == 41 && (state.shape != 2 || !tap(&host, 'v') || !tap(&host, 'n')))
                goto done;
            if (frame == 43 && (!wire || !state.lanes[1].hits || !tap(&host, '1')))
                goto done;
            if (frame == 45 && (state.shape != 0 || !tap(&host, 'n')))
                goto done;
            if (frame == 47 && (!state.lanes[1].hits || !tap(&host, '2') || !tap(&host, 'r')))
                goto done;
            if (frame == 49 &&
                (state.steps != 0 || state.shape != 1 || !tap(&host, YMGRE_KEY_ESCAPE)))
                goto done;
        }
        unsigned a = YMGRE_DemoHost_TakeActions(&host);
        if (a & QUIT)
        {
            result = 0;
            break;
        }
        if (a & PAUSE)
        {
            paused = !paused;
            accumulator = 0;
        }
        if (a & WIRE)
            wire = !wire;
        if (a & (RESET | BOUNCE | PLANE | BOX | MESH))
        {
            int shape = a & PLANE  ? 0
                        : a & BOX  ? 1
                        : a & MESH ? 2
                                   : state.shape,
                bounce = a & BOUNCE ? !state.bounce : state.bounce;
            if (!CCDLab_Init(&state, shape, bounce))
                goto done;
            accumulator = 0;
        }
        if (paused && (a & STEP) && !CCDLab_Step(&state))
            goto done;
        if (!paused)
        {
            accumulator += fmin(elapsed, .05);
            if (accumulator >= .5)
            {
                accumulator -= .5;
                if (state.steps >= 3)
                {
                    if (!CCDLab_Init(&state, state.shape, state.bounce))
                        goto done;
                }
                else if (!CCDLab_Step(&state))
                    goto done;
            }
        }
        CCDScene_Render(&scene, &state, wire);
        YMGUI_Obj_Invalidate(target);
        char text[200];
        const char *names[] = {"PLANE (infinite)", "BOX (20 mm thick)",
                               "CONVEX MESH (sheared thin wall)"};
        snprintf(text, sizeof text, "TOP orange: DISCRETE   |   BOTTOM green: CCD   |   %s",
                 names[state.shape]);
        YMGUI_Label_SetText(rows[0], text);
        snprintf(
            text, sizeof text,
            "100 m/s | radius 100 mm | dt 20 ms (2 m travel) | display slowed 25x | %s | step %u",
            paused ? "PAUSED" : "RUN", state.steps);
        YMGUI_Label_SetText(rows[1], text);
        snprintf(text, sizeof text,
                 "Discrete x %+.3f m   |   CCD x %+.3f m / vx %+.2f m/s   |   restitution %d",
                 state.lanes[0].bodies[1].position.x, state.lanes[1].bodies[1].position.x,
                 state.lanes[1].bodies[1].velocity.x, state.bounce);
        YMGUI_Label_SetText(rows[2], text);
        snprintf(text, sizeof text,
                 "CCD impacts %u | latest TOI %.3f ms | total normal impulse %.2f Ns | capped "
                 "bodies %zu",
                 state.lanes[1].hits, state.lanes[1].impactTime * 1000, state.lanes[1].impulse,
                 state.lanes[1].ccd.clampedBodies);
        YMGUI_Label_SetText(rows[3], text);
        YMGUI_Label_SetText(rows[4], "SPACE Pause | N Single step | R Reset | B Bounce | 1 Plane / "
                                     "2 Box / 3 Convex | V Wire | ESC Exit");
        YMGUI_Label_SetText(rows[5], "Line: start-to-current position. Yellow: impact normal. "
                                     "Infinite plane shown as a finite outline.");
        if (!YMGRE_DemoHost_Step(&host, smoke ? 0 : 60))
        {
            result = smoke ? 1 : 0;
            break;
        }
    }
done:
    if (ready)
        YMGRE_DemoHost_Destroy(&host);
    CCDScene_Destroy(&scene);
    if (smoke)
        printf("CCD_WINDOW %s\n", result ? "FAIL" : "PASS");
    return result;
}
