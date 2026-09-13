#define _POSIX_C_SOURCE 200809L
#include "ball.h"
#include "demo_host.h"
#include "YMGUI_Label.h"
#include "YMGUI_Button.h"
#include "YMGUI_Event.h"
#include "YMGUI_Invalidate.h"
#include "YMGUI_Image.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
static hand_ball_scene scene;
static int running = 1, paused, failed;
static GYOBJ rows[12], view, uvView, uvBar, uvTitle, uvScale, uvInfo, buttons[16];
enum
{
	UV_X = 672,
	UV_Y = 42
};
static const struct
{
	int key;
	const char *label;
} controls[] = {{'f', "F GRIP"},		 {'o', "O OPEN"},
				{'+', "+ TIGHTER"},		 {'-', "- LOOSER"},
				{'w', "W SOLID / WIRE"}, {'m', "M MOVE HAND"},
				{'p', "P NUDGE BALL"},	 {'v', "V GRE / SOURCE UV"},
				{'h', "H SKIN HEAT"},	 {'t', "T RGB / |F| / Pa"},
				{'a', "A FORCE ARROWS"}, {'c', "C SPIN BALL"},
				{'z', "Z TARE"},		 {'r', "R RESET"},
				{'g', "G GRAVITY"},		 {' ', "SPACE PAUSE"}};
static float norm(pme_vec3 v) { return sqrtf(YMPME_VecDot(v, v)); }
static double seconds(void)
{
	struct timespec t;
	clock_gettime(CLOCK_MONOTONIC, &t);
	return t.tv_sec + t.tv_nsec * 1e-9;
}
static void command(int key)
{
	if (key >= 'A' && key <= 'Z')
		key += 'a' - 'A';
	switch (key)
	{
	case 27:
		running = 0;
		break;
	case ' ':
		paused = !paused;
		break;
	case 'f':
		scene.grip = 1;
		break;
	case 'o':
		scene.grip = 0;
		break;
	case '+':
	case '=':
		scene.grip = fminf(1, scene.grip + .1f);
		break;
	case '-':
		scene.grip = fmaxf(0, scene.grip - .1f);
		break;
	case 'a':
		scene.arrows = !scene.arrows;
		break;
	case 'c':
		YMPME_Rigid_Impulse(&scene.ball, YMPME_Vec3(0, 0, 0), YMPME_Vec3(0, 0, .00002f));
		break;
	case 'w':
		scene.wire = !scene.wire;
		break;
	case 'm':
		scene.motion = !scene.motion;
		break;
	case 'p':
		HandBall_Nudge(&scene, YMPME_Vec3(.003f, 0, .003f));
		break;
	case 'v':
		scene.sourceUv = !scene.sourceUv;
		scene.skin.uvSelected = -1;
		break;
	case 'h':
		scene.skin.heat = !scene.skin.heat;
		break;
	case 't':
		scene.colorMode = (scene.colorMode + 1) % 3;
		scene.skin.uvForce = scene.colorMode == 2 ? 0 : 2;
		scene.skin.vectorHeat = scene.skin.uvForce == 2;
		break;
	case 'b':
		scene.skin.bones = !scene.skin.bones;
		break;
	case 'g':
		scene.gravity = !scene.gravity;
		break;
	case 'q':
		scene.tilt = fmaxf(-1.3f, scene.tilt - .15f);
		break;
	case 'e':
		scene.tilt = fminf(1.3f, scene.tilt + .15f);
		break;
	case 'j':
		scene.wristTarget.x = fmaxf(-.07f, scene.wristTarget.x - .005f);
		break;
	case 'l':
		scene.wristTarget.x = fminf(.07f, scene.wristTarget.x + .005f);
		break;
	case 'i':
		scene.wristTarget.y = fminf(.045f, scene.wristTarget.y + .005f);
		break;
	case 'k':
		scene.wristTarget.y = fmaxf(-.045f, scene.wristTarget.y - .005f);
		break;
	case 'u':
		scene.wristTarget.z = fminf(.07f, scene.wristTarget.z + .005f);
		break;
	case 'n':
		scene.wristTarget.z = fmaxf(0, scene.wristTarget.z - .005f);
		break;
	case 'z':
		YMPME_ForceDevice_TareNext(&scene.skin.loadCell);
		scene.skin.relative = 1;
		break;
	case 'r':
		if (!HandBall_Reset(&scene))
			failed = 1;
		scene.wire = 0;
		scene.sourceUv = 0;
		scene.skin.uvForce = 2;
		scene.skin.vectorHeat = 1;
		scene.arrows = 0;
		scene.colorMode = 0;
		scene.skin.relative = 0;
		scene.skin.heat = 1;
		scene.skin.bones = 0;
		scene.skin.uvSelected = -1;
		paused = 0;
		break;
	case GY_KEY_LEFT:
		Demo_ViewMove(&scene.skin.hand, -.15f, 0, 0);
		break;
	case GY_KEY_RIGHT:
		Demo_ViewMove(&scene.skin.hand, .15f, 0, 0);
		break;
	case GY_KEY_UP:
		Demo_ViewMove(&scene.skin.hand, 0, .1f, 0);
		break;
	case GY_KEY_DOWN:
		Demo_ViewMove(&scene.skin.hand, 0, -.1f, 0);
		break;
	}
}
static void event(GYOBJ obj, GYEvent e)
{
	if (e == GY_EVENT_Key)
	{
		command((int)obj->ctx->last_key);
		obj->ctx->key_handled = 1;
	}
	if (e == GY_EVENT_Wheel)
		Demo_ViewMove(&scene.skin.hand, 0, 0, -obj->ctx->wheel_y * .09f);
}
static void uvEvent(GYOBJ obj, GYEvent e)
{
	if (e == GY_EVENT_Pressed)
	{
		GYrect a;
		YMGUI_Obj_GetAbsArea(obj, &a);
		scene.skin.uvSelected =
			HandUV_Pick(HandBall_UV(&scene), obj->ctx->point_x - a.x, obj->ctx->point_y - a.y);
	}
	event(obj, e);
}
static void clicked(GYOBJ obj)
{
	for (int i = 0; i < 16; i++)
		if (buttons[i] == obj)
			command(controls[i].key);
}
static GYOBJ label(GYOBJ root, int x, int y, int width, const char *text)
{
	GYOBJ o = YMGUI_Creat_Label_Creat(root, x, y, width, 22);
	if (o)
	{
		YMGUI_Label_SetText(o, text);
		YMGUI_Label_SetTextColor(o, GY_ARGB(255, 230, 235, 243));
	}
	return o;
}
static void refresh(void)
{
	char t[128];
	hand_skin_rig *s = &scene.skin;
	snprintf(t, sizeof t, "Grip %.0f%% | %s", scene.grip * 100, paused ? "PAUSED" : "RUNNING");
	YMGUI_Label_SetText(rows[0], t);
	snprintf(t, sizeof t, "Ball: 128 faces | 64 mm | 35 g");
	YMGUI_Label_SetText(rows[1], t);
	snprintf(t, sizeof t, "Skin normal total: %.3f N", scene.totalNormal);
	YMGUI_Label_SetText(rows[2], t);
	snprintf(t, sizeof t, "Peak %.0f Pa | %zu contacts", s->peakPressure, s->reaction.contacts);
	YMGUI_Label_SetText(rows[3], t);
	snprintf(t, sizeof t, "Contact |F| %s: %.3f N", s->relative ? "REL" : "ABS",
			 norm(s->loadReading.sample.wrench.force));
	YMGUI_Label_SetText(rows[4], t);
	snprintf(t, sizeof t, "Contact |T|: %.5f Nm", norm(s->loadReading.sample.wrench.torque));
	YMGUI_Label_SetText(rows[5], t);
	snprintf(t, sizeof t, "Speed %.3f m/s | Spin %.2f rad/s", norm(scene.ball.velocity),
			 norm(scene.ball.angularVelocity));
	YMGUI_Label_SetText(rows[6], t);
	snprintf(t, sizeof t, "Depth %.2f mm | cap %s", s->reaction.maxPenetration * 1000,
			 s->reaction.saturated ? "ON" : "OFF");
	YMGUI_Label_SetText(rows[7], t);
	snprintf(t, sizeof t, "Gravity %s | table %.3f N", scene.gravity ? "ON" : "OFF",
			 scene.floorForce);
	YMGUI_Label_SetText(rows[8], t);
	snprintf(t, sizeof t, "Move %s | tilt %+.0f deg", scene.motion ? "ON" : "OFF",
			 scene.tilt * 57.29578f);
	YMGUI_Label_SetText(rows[9], t);
	snprintf(t, sizeof t, "%.2f s | 240 Hz | %s", s->time,
			 scene.lost ? "BALL AWAY: R" : "FREE BODY");
	YMGUI_Label_SetText(rows[10], t);
	snprintf(t, sizeof t, "GRE UV: %d islands | RGB clip %zu", scene.autoCharts, scene.rgbClipped);
	YMGUI_Label_SetText(rows[11], t);
	snprintf(t, sizeof t, "%s / %s", scene.sourceUv ? "SOURCE UV" : "GRE AUTO UV",
			 scene.colorMode == 0 ? "RGB / R NORMAL, G U-SHEAR, B V-SHEAR"
			 : s->uvForce		  ? "3D FORCE MAGNITUDE (N)"
								  : "SKIN PRESSURE (Pa)");
	YMGUI_Label_SetText(uvTitle, t);
	YMGUI_Label_SetText(uvScale, scene.colorMode == 0 ? "R: 0..0.10 N | G: +/-0.05 N | B: +/-0.05 N"
								 : s->uvForce
									 ? "0                 0.05                 0.10 N |F| / patch"
									 : "0                  500                   1000 Pa");
	if (s->uvSelected >= 0)
	{
		pme_surface_taxel *c = &s->taxels[s->uvSelected];
		pme_vec3 local = {0};
		HandBall_LocalForce(&scene, (size_t)s->uvSelected, &local);
		snprintf(t, sizeof t, "%d: |F| %.3f N / U%+.3f V%+.3f N%.3f", s->uvSelected, norm(c->force),
				 local.x, local.y, local.z);
	}
	else
		snprintf(t, sizeof t, "%s",
				 s->uvSelected == -2	? "Overlapping source UV: ambiguous surface"
				 : scene.colorMode == 0 ? "G/B: 128=zero | >128 positive | <128 negative | A arrows"
										: "24px grid: UP=normal | tip offset=U/V shear | A arrows");
	YMGUI_Label_SetText(uvInfo, t);
}
static void clickButton(int i)
{
	GYrect a;
	YMGUI_Obj_GetAbsArea(buttons[i], &a);
	YMGUI_Inject_Pointer(a.x + 10, a.y + 10, 1);
	YMGUI_Inject_Pointer(a.x + 10, a.y + 10, 0);
}
int main(int argc, char **argv)
{
	int smoke = argc == 2 && !strcmp(argv[1], "--smoke-test"),
		headless = argc == 2 && !strcmp(argv[1], "--headless");
	int ok = 0, ready = 0;
	YMGRE_DemoHost host;
	if (argc > 1 && !smoke && !headless)
		return 2;
	if (!HandBall_Init(&scene))
		goto done;
	scene.motion = 1;
	if (headless)
	{
		for (int i = 0; i < 480; i++)
			if (!HandBall_Step(&scene))
				goto done;
		HandBall_Render(&scene);
		ok = Demo_Save(&scene.skin.hand, "hand_ball.ppm");
		goto done;
	}
	YMGRE_DemoHost_DefaultScale(1);
	if (!YMGRE_DemoHost_Init(&host, 1280, 800, 40))
		goto done;
	ready = 1;
	view = YMGRE_DemoHost_AddTarget(&host, YMGRE_Camera_GetRenderTarget(scene.skin.hand.camera), 0,
									42, 640, 426);
	uvView = YMGRE_DemoHost_AddTarget(&host, YMGRE_Camera_GetRenderTarget(scene.skin.uvCamera),
									  UV_X, UV_Y, HAND_UV_SIZE, HAND_UV_SIZE);
	uvBar = YMGRE_DemoHost_AddTarget(&host, YMGRE_Camera_GetRenderTarget(scene.skin.uvLegend), UV_X,
									 572, HAND_UV_SIZE, 16);
	uvTitle = label(host.context->root, UV_X, 8, HAND_UV_SIZE, "");
	uvScale = label(host.context->root, UV_X, 596, HAND_UV_SIZE, "");
	uvInfo = label(host.context->root, UV_X, 622, 600, "");
	if (!view || !uvView || !uvBar || !uvTitle || !uvScale || !uvInfo)
		goto done;
	YMGUI_Image_SetScaleMode(view, GY_IMG_FIT);
	view->state |= GY_STATE_Focusable;
	view->event_cb = event;
	uvView->event_cb = uvEvent;
	if (!label(host.context->root, 0, 8, 640, "HAND BALL / GRIP - MOVE - RELEASE"))
		goto done;
	for (int i = 0; i < 12; i++)
	{
		rows[i] = label(host.context->root, (i / 6) * 320, 480 + (i % 6) * 25, 320, "");
		if (!rows[i])
			goto done;
	}
	for (int i = 0; i < 16; i++)
	{
		buttons[i] = YMGUI_Creat_Button_Creat(host.context->root, 4 + (i % 8) * 160,
											  654 + (i / 8) * 34, 152, 28);
		if (!buttons[i])
			goto done;
		YMGUI_Button_SetText(buttons[i], controls[i].label);
		YMGUI_Button_SetClicked(buttons[i], clicked);
	}
	if (!label(host.context->root, 0, 727, 1280,
			   "F/O Grip/Open | W Wire | P Nudge | C Spin | M Move | SPACE Pause") ||
		!label(host.context->root, 0, 752, 1280,
			   "J/L X | I/K Y | U/N Z | Q/E Tilt | ARROWS View | WHEEL Zoom") ||
		!label(host.context->root, 0, 777, 1280,
			   "A Arrows | V UV | T RGB/|F|/Pa | B Bones | R Reset | ESC Exit"))
		goto done;
	double previous = seconds(), accumulator = 0;
	for (unsigned frame = 0; running && !failed; frame++)
	{
		YMGUI_SetFocus(host.context, view);
		if (smoke)
		{
			if (frame == 5)
				clickButton(4);
			if (frame == 6 && !scene.wire)
				goto done;
			if (frame == 10)
				YMGUI_Inject_Key('o', 1);
			if (frame == 11 && scene.grip != 0)
				goto done;
			if (frame == 15)
				clickButton(0);
			if (frame == 16 && scene.grip != 1)
				goto done;
			if (frame == 20)
				clickButton(7);
			if (frame == 21 && !scene.sourceUv)
				goto done;
			if (frame == 25)
				YMGUI_Inject_Key('v', 1);
			if (frame == 26 && scene.sourceUv)
				goto done;
			if (frame == 27)
				clickButton(10);
			if (frame == 28 && !scene.arrows)
				goto done;
			if (frame == 29)
				clickButton(11);
			if (frame == 30)
				YMGUI_Inject_Key('u', 1);
			if (frame == 31 && scene.wristTarget.z <= 0)
				goto done;
			if (frame == 35)
				clickButton(6);
			if (frame == 40)
				clickButton(13);
			if (frame == 41 && (scene.wire || scene.sourceUv || scene.wristTarget.z != 0))
				goto done;
			if (frame == 70)
			{
				const hand_uv_map *uv = HandBall_UV(&scene);
				int found = -1;
				for (int p = 0; p < HAND_UV_SIZE * HAND_UV_SIZE; p++)
				{
					int cell = HandUV_Pick(uv, p % HAND_UV_SIZE, p / HAND_UV_SIZE);
					if (cell >= 0 && scene.skin.taxels[cell].pressure > 0)
					{
						found = p;
						break;
					}
				}
				if (found < 0)
					goto done;
				YMGUI_Inject_Pointer(UV_X + found % HAND_UV_SIZE, UV_Y + found / HAND_UV_SIZE, 1);
				YMGUI_Inject_Pointer(UV_X + found % HAND_UV_SIZE, UV_Y + found / HAND_UV_SIZE, 0);
				if (scene.skin.uvSelected < 0)
					goto done;
			}
			if (frame == 72)
				clickButton(9);
			if (frame == 73 && scene.colorMode != 1)
				goto done;
			if (frame == 74)
				clickButton(9);
			if (frame == 75 && scene.colorMode != 2)
				goto done;
			if (frame == 76)
				clickButton(9);
			if (frame == 77 && scene.colorMode != 0)
				goto done;
			if (frame == 80)
			{
				ok = scene.skin.touch.stamp.valid && scene.skin.reaction.contacts > 0;
				break;
			}
		}
		double now = seconds(), elapsed = smoke ? 1. / 60 : fmin(.05, now - previous);
		previous = now;
		if (paused)
			accumulator = 0;
		else
		{
			accumulator += elapsed;
			while (accumulator >= 1. / 240)
			{
				if (!HandBall_Step(&scene))
					goto done;
				accumulator -= 1. / 240;
			}
		}
		HandBall_Render(&scene);
		refresh();
		YMGUI_Obj_Invalidate(view);
		YMGUI_Obj_Invalidate(uvView);
		YMGUI_Obj_Invalidate(uvBar);
		if (!YMGRE_DemoHost_Step(&host, smoke ? 0 : 60))
		{
			ok = 1;
			break;
		}
	}
	if (!running && !failed)
		ok = 1;
done:
	if (ready)
		YMGRE_DemoHost_Destroy(&host);
	HandBall_Destroy(&scene);
	if (smoke)
		printf("HAND_BALL_WINDOW %s\n", ok ? "PASS" : "FAIL");
	return !ok;
}
