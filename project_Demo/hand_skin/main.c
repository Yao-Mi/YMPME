#define _POSIX_C_SOURCE 200809L
#include "rig.h"
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
static double seconds(void)
{
	struct timespec t;
	clock_gettime(CLOCK_MONOTONIC, &t);
	return t.tv_sec + t.tv_nsec * 1e-9;
}
static hand_skin_rig rig;
static int running = 1, paused = 0, failed = 0;
static GYOBJ rows[12], view, uvView, uvBar, uvTitle, uvScale, uvInfo;
enum
{
	UV_X = 672,
	UV_Y = 42
};
static void command(int key)
{
	if (key >= '0' && key <= '5')
	{
		if (!HandSkin_Select(&rig, key - '0'))
			failed = 1;
		return;
	}
	switch (key)
	{
	case 27:
		running = 0;
		break;
	case ' ':
		paused = !paused;
		break;
	case '+':
	case '=':
		rig.indentation = fminf(.006f, rig.indentation + .0005f);
		break;
	case '-':
		rig.indentation = fmaxf(-.01f, rig.indentation - .0005f);
		break;
	case 't':
	case 'T':
		rig.uvForce = !rig.uvForce;
		break;
	case 'i':
	case 'I':
		rig.noise = !rig.noise;
		break;
	case 'h':
	case 'H':
		rig.heat = !rig.heat;
		break;
	case 'b':
	case 'B':
		rig.bones = !rig.bones;
		break;
	case 'z':
	case 'Z':
		YMPME_ForceDevice_TareNext(&rig.loadCell);
		rig.relative = 1;
		break;
	case 'o':
	case 'O':
		for (int i = 0; i < 5; ++i)
			Demo_SetFingerCurl(&rig.hand, i, 0);
		break;
	case 'f':
	case 'F':
		for (int i = 0; i < 5; ++i)
			Demo_SetFingerCurl(&rig.hand, i, .5f);
		break;
	case 'r':
	case 'R':
		rig.indentation = .003f;
		rig.noise = 0;
		rig.relative = 0;
		rig.heat = 1;
		rig.uvForce = 0;
		rig.uvSelected = -1;
		paused = 0;
		{
			pme_force_device_desc d = rig.loadCell.desc;
			d.forceNoise = d.torqueNoise = 0;
			if (YMPME_ForceDevice_Init(&rig.loadCell, &d, 734) != PME_OK ||
				!HandSkin_Select(&rig, 0))
				failed = 1;
		}
		break;
	case GY_KEY_LEFT:
		Demo_ViewMove(&rig.hand, -.15f, 0, 0);
		break;
	case GY_KEY_RIGHT:
		Demo_ViewMove(&rig.hand, .15f, 0, 0);
		break;
	case GY_KEY_UP:
		Demo_ViewMove(&rig.hand, 0, .1f, 0);
		break;
	case GY_KEY_DOWN:
		Demo_ViewMove(&rig.hand, 0, -.1f, 0);
		break;
	}
}
static void event(GYOBJ obj, GYEvent event)
{
	if (event == GY_EVENT_Key)
	{
		command((int)obj->ctx->last_key);
		obj->ctx->key_handled = 1;
	}
	if (event == GY_EVENT_Wheel)
		Demo_ViewMove(&rig.hand, 0, 0, -obj->ctx->wheel_y * .09f);
}
static void uv_event(GYOBJ obj, GYEvent e)
{
	if (e == GY_EVENT_Pressed)
	{
		GYrect a;
		YMGUI_Obj_GetAbsArea(obj, &a);
		rig.uvSelected = HandUV_Pick(&rig.uv, obj->ctx->point_x - a.x, obj->ctx->point_y - a.y);
	}
	if (e == GY_EVENT_Key)
	{
		command((int)obj->ctx->last_key);
		obj->ctx->key_handled = 1;
	}
}
static GYOBJ buttons[14];
static void clicked(GYOBJ obj)
{
	const char *keys = "012345+-hzroft";
	for (int i = 0; i < 14; ++i)
		if (buttons[i] == obj)
			command(keys[i]);
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
	const char *names[] = {"PALM", "THUMB", "INDEX", "MIDDLE", "RING", "PINKY"};
	char text[64];
	snprintf(text, sizeof text, "Target: %s | %s", names[rig.region],
			 paused ? "PAUSED" : "RUNNING");
	YMGUI_Label_SetText(rows[0], text);
	snprintf(text, sizeof text, "Indent target: %+.1f mm", rig.indentation * 1000);
	YMGUI_Label_SetText(rows[1], text);
	snprintf(text, sizeof text, "Load %s: %+.4f N", rig.relative ? "REL" : "ABS", rig.normalLoad);
	YMGUI_Label_SetText(rows[2], text);
	snprintf(text, sizeof text, "Peak pressure: %.1f Pa", rig.peakPressure);
	YMGUI_Label_SetText(rows[3], text);
	snprintf(text, sizeof text, "Contact patches: %zu / %zu", rig.reaction.contacts, rig.count);
	YMGUI_Label_SetText(rows[4], text);
	snprintf(text, sizeof text, "Max penetration: %.2f mm", rig.reaction.maxPenetration * 1000);
	YMGUI_Label_SetText(rows[5], text);
	snprintf(text, sizeof text, "Probe torque Z: %+.5f Nm", rig.loadReading.sample.wrench.torque.z);
	YMGUI_Label_SetText(rows[6], text);
	int joint = rig.region ? rig.region * 3 - 1 : 3;
	snprintf(text, sizeof text, "J%02d target %.1f / actual %.1f deg", joint + 1,
			 Demo_TargetAngle(&rig.hand, joint) * 57.29578f,
			 rig.hand.joints[joint].coordinate * 57.29578f);
	YMGUI_Label_SetText(rows[7], text);
	snprintf(text, sizeof text, "Joint reaction |F|: %.4f N",
			 sqrtf(YMPME_VecDot(rig.hand.joints[joint].feedback.reactionB.force,
								rig.hand.joints[joint].feedback.reactionB.force)));
	YMGUI_Label_SetText(rows[8], text);
	snprintf(text, sizeof text, "Noise %s | pressure cap %s", rig.noise ? "ON" : "OFF",
			 rig.reaction.saturated ? "ACTIVE" : "OFF");
	YMGUI_Label_SetText(rows[9], text);
	snprintf(text, sizeof text, "Time %.2f s | physics 240 Hz", rig.time);
	YMGUI_Label_SetText(rows[10], text);
	snprintf(text, sizeof text, "3D 0-1000 Pa | UV overlap %zu px", rig.uv.overlapPixels);
	YMGUI_Label_SetText(rows[11], text);
	YMGUI_Label_SetText(uvTitle, rig.uvForce ? "UV FORCE / NORMAL FORCE PER PATCH (N)"
											 : "UV PRESSURE / UNWRAPPED SKIN (Pa)");
	YMGUI_Label_SetText(uvScale, rig.uvForce
									 ? "0                 0.05                 0.10 N / patch"
									 : "0                  500                   1000 Pa");
	if (rig.uvSelected >= 0)
	{
		const pme_surface_taxel *c = &rig.taxels[rig.uvSelected];
		snprintf(text, sizeof text, "Patch %d | %.1f Pa | %.4f N | %.3f cm2", rig.uvSelected,
				 c->pressure, c->normalForce, rig.patches[rig.uvSelected].area * 10000);
		YMGUI_Label_SetText(uvInfo, text);
	}
	else
	{
		snprintf(text, sizeof text, "Click UV | magenta=overlap | %zu subpixel patch",
				 rig.uv.uncoveredPatches);
		YMGUI_Label_SetText(uvInfo, rig.uvSelected == -2
										? "Overlapping UV: ambiguous surface, no unique readout"
										: text);
	}
}
int main(int argc, char **argv)
{
	int smoke = argc == 2 && !strcmp(argv[1], "--smoke-test");
	int headless = argc == 2 && !strcmp(argv[1], "--headless");
	int ok = 0, ready = 0;
	YMGRE_DemoHost host;
	if (argc > 1 && !smoke && !headless)
		return 2;
	if (!HandSkin_Init(&rig))
		goto done;
	if (headless)
	{
		for (int i = 0; i < 120; ++i)
			if (!HandSkin_Step(&rig))
				goto done;
		HandSkin_Render(&rig);
		ok = Demo_Save(&rig.hand, "hand_skin.ppm");
		goto done;
	}
	YMGRE_DemoHost_DefaultScale(1);
	if (!YMGRE_DemoHost_Init(&host, 1280, 800, 40))
		goto done;
	ready = 1;
	view = YMGRE_DemoHost_AddTarget(&host, YMGRE_Camera_GetRenderTarget(rig.hand.camera), 0, 42,
									640, 426);
	if (!view)
		goto done;
	YMGUI_Image_SetScaleMode(view, GY_IMG_FIT);
	uvView = YMGRE_DemoHost_AddTarget(&host, YMGRE_Camera_GetRenderTarget(rig.uvCamera), UV_X, UV_Y,
									  HAND_UV_SIZE, HAND_UV_SIZE);
	uvBar = YMGRE_DemoHost_AddTarget(&host, YMGRE_Camera_GetRenderTarget(rig.uvLegend), UV_X, 572,
									 HAND_UV_SIZE, 16);
	uvTitle = label(host.context->root, UV_X, 8, HAND_UV_SIZE, "");
	uvScale = label(host.context->root, UV_X, 596, HAND_UV_SIZE, "");
	uvInfo = label(host.context->root, UV_X, 622, HAND_UV_SIZE, "");
	if (!uvView || !uvBar || !uvTitle || !uvScale || !uvInfo)
		goto done;
	uvView->event_cb = uv_event;
	view->state |= GY_STATE_Focusable;
	view->event_cb = event;
	if (!label(host.context->root, 0, 8, 640, "HAND LOW / 3D PRESSURE"))
		goto done;
	for (int i = 0; i < 12; ++i)
	{
		rows[i] = label(host.context->root, (i / 6) * 320, 480 + (i % 6) * 25, 320, "");
		if (!rows[i])
			goto done;
	}
	const char *names[] = {"0 PALM",  "1 THUMB", "2 INDEX",	  "3 MIDDLE", "4 RING",
						   "5 PINKY", "+ PRESS", "- RELEASE", "H HEAT",	  "Z TARE",
						   "R RESET", "O OPEN",	 "F CURL",	  "T Pa / N"};
	for (int i = 0; i < 14; ++i)
	{
		buttons[i] = YMGUI_Creat_Button_Creat(host.context->root, 4 + (i % 7) * 182,
											  654 + (i / 7) * 34, 174, 28);
		if (!buttons[i])
			goto done;
		YMGUI_Button_SetText(buttons[i], names[i]);
		YMGUI_Button_SetClicked(buttons[i], clicked);
	}
	if (!label(host.context->root, 0, 727, 1280,
			   "0-5 Target | +/- Depth | H Heat | I Noise | Z Tare | SPACE Pause") ||
		!label(host.context->root, 0, 752, 1280,
			   "ARROWS Orbit | WHEEL Zoom | B Bones | O/F Open/Curl | R Reset") ||
		!label(host.context->root, 0, 777, 1280,
			   "T UV Pa/N | CLICK UV Readout | ESC Exit | Approximate contact"))
		goto done;
	double previous = seconds(), accumulator = 0;
	for (unsigned frame = 0; running && !failed; ++frame)
	{
		YMGUI_SetFocus(host.context, view);
		if (smoke)
		{
			if (frame == 5)
				YMGUI_Inject_Key('+', 1);
			if (frame == 6 && rig.indentation <= .003f)
				goto done;
			if (frame == 10)
			{
				GYrect a;
				YMGUI_Obj_GetAbsArea(buttons[2], &a);
				YMGUI_Inject_Pointer(a.x + 10, a.y + 10, 1);
				YMGUI_Inject_Pointer(a.x + 10, a.y + 10, 0);
			}
			if (frame == 11 && rig.region != 2)
				goto done;
			if (frame == 15)
				YMGUI_Inject_Key('z', 1);
			if (frame == 16 && !rig.relative)
				goto done;
			if (frame == 20)
				YMGUI_Inject_Key('i', 1);
			if (frame == 21 && !rig.noise)
				goto done;
			if (frame == 25)
				YMGUI_Inject_Key('h', 1);
			if (frame == 26 && rig.heat)
				goto done;
			if (frame == 30)
				YMGUI_Inject_Key('r', 1);
			if (frame == 32)
			{
				int pixel = -1;
				for (int k = 0; k < HAND_UV_SIZE * HAND_UV_SIZE; ++k)
				{
					int p = HandUV_Pick(&rig.uv, k % HAND_UV_SIZE, k / HAND_UV_SIZE);
					if (p >= 0 && rig.taxels[p].pressure > 0)
					{
						pixel = k;
						break;
					}
				}
				if (pixel < 0)
					goto done;
				int expected = HandUV_Pick(&rig.uv, pixel % HAND_UV_SIZE, pixel / HAND_UV_SIZE);
				YMGUI_Inject_Pointer(UV_X + pixel % HAND_UV_SIZE, UV_Y + pixel / HAND_UV_SIZE, 1);
				YMGUI_Inject_Pointer(UV_X + pixel % HAND_UV_SIZE, UV_Y + pixel / HAND_UV_SIZE, 0);
				if (rig.uvSelected != expected)
					goto done;
			}
			if (frame == 33)
				YMGUI_Inject_Key('t', 1);
			if (frame == 34 && !rig.uvForce)
				goto done;
			if (frame == 35 && (!rig.heat || rig.noise || rig.relative || rig.region != 0))
				goto done;
			if (frame == 40)
			{
				ok = rig.touch.stamp.valid && rig.reaction.contacts > 0;
				break;
			}
		}
		double now = seconds(), elapsed = smoke ? 1.0 / 60 : fmin(.05, now - previous);
		previous = now;
		if (paused)
			accumulator = 0;
		else
		{
			accumulator += elapsed;
			while (accumulator >= 1.0 / 240)
			{
				if (!HandSkin_Step(&rig))
					goto done;
				accumulator -= 1.0 / 240;
			}
		}
		HandSkin_Render(&rig);
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
	HandSkin_Destroy(&rig);
	if (smoke)
		printf("HAND_SKIN_WINDOW %s\n", ok ? "PASS" : "FAIL");
	return !ok;
}
