#define _POSIX_C_SOURCE 200809L
#include "rig.h"
#include "observation.h"
#include "demo_host.h"
#include "YMGUI_Label.h"
#include "YMGUI_Button.h"
#include "YMGUI_Event.h"
#include "YMGUI_Image.h"
#include "YMGUI_Invalidate.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
static human_skin_rig rig;
static GYOBJ view, uv, bar, rows[10], scale, info, buttons[8];
static int running = 1, paused, failed;
static const char *keys = "1234 wb ";
static const char *names[] = {"1 WALK",		 "2 STAND",		"3 SQUAT", "4 AIR",
							  "SPACE PAUSE", "W WIREFRAME", "B BONES", "PICK TEST OFF"};
static double seconds(void)
{
	struct timespec t;
	clock_gettime(CLOCK_MONOTONIC, &t);
	return t.tv_sec + t.tv_nsec * 1e-9;
}
static void focusFoot(int side)
{
	int peak = -1;
	for (size_t p = 0; p < rig.count; p++)
		if (rig.ground.foot[p] == side && (peak < 0 || rig.forces[p].y > rig.forces[peak].y))
			peak = (int)p;
	if (peak < 0)
		return;
	HumanContactView_FocusFace(&rig, peak);
}
static void command(int key)
{
	if (key >= 'A' && key <= 'Z')
		key += 'a' - 'A';
	if (key >= '1' && key <= '4')
	{
		if (!HumanGround_Enable(&rig, key - '1'))
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
	case 'w':
		rig.wire = !rig.wire;
		break;
	case 'b':
		rig.bones = !rig.bones;
		break;
	case 'h':
		rig.heat = !rig.heat;
		break;
	case 't':
		rig.colorMode = (rig.colorMode + 1) % 3;
		break;
	case 'c':
		rig.gain = rig.gain == 10 ? 2 : rig.gain == 2 ? 40 : 10;
		break;
	case 'y':
		rig.contactView.showGroups = !rig.contactView.showGroups;
		break;
	case 'v':
		rig.contactView.overview = rig.contactView.overview == 3 ? 1 : 3;
		rig.uvZoom = 1;
		rig.uvCenterU = rig.uvCenterV = .5f;
		break;
	case 'u':
		rig.uvZoom = 1;
		rig.uvCenterU = rig.uvCenterV = .5f;
		rig.selectedUv = -1;
		break;
	case 'l':
		focusFoot(1);
		break;
	case 'r':
		focusFoot(2);
		break;
	case GY_KEY_LEFT:
		Demo_ViewMove(&rig.human, -.15f, 0, 0);
		break;
	case GY_KEY_RIGHT:
		Demo_ViewMove(&rig.human, .15f, 0, 0);
		break;
	case GY_KEY_UP:
		Demo_ViewMove(&rig.human, 0, .1f, 0);
		break;
	case GY_KEY_DOWN:
		Demo_ViewMove(&rig.human, 0, -.1f, 0);
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
	GYrect a;
	YMGUI_Obj_GetAbsArea(obj, &a);
	float x = obj->ctx->point_x - a.x, y = obj->ctx->point_y - a.y;
	if (e == GY_EVENT_Wheel)
	{
		if (obj == uv)
			HumanSkin_ZoomUV(&rig, x, y, expf(obj->ctx->wheel_y * .25f));
		else
			Demo_ViewMove(&rig.human, 0, 0, -obj->ctx->wheel_y * .09f);
	}
	if (e == GY_EVENT_Pressed)
	{
		rig.selectedUv = obj == uv ? HumanSkin_PickUV(&rig, x, y)
								   : HumanSkin_Pick3D(&rig, x * 1.5f, y * 640.f / 426);
	}
}
static GYOBJ label(GYOBJ root, int x, int y, int w, const char *text)
{
	GYOBJ o = YMGUI_Creat_Label_Creat(root, x, y, w, 24);
	if (o)
	{
		YMGUI_Label_SetText(o, text);
		YMGUI_Label_SetTextColor(o, GY_ARGB(255, 230, 235, 243));
	}
	return o;
}
static void clicked(GYOBJ o)
{
	for (int i = 0; i < 8; i++)
		if (buttons[i] == o)
		{
			if (i == 7)
				rig.pickTest = !rig.pickTest;
			else
				command(keys[i]);
		}
}
static void refresh(void)
{
	YMGUI_Button_SetText(buttons[7], rig.pickTest ? "PICK TEST ON" : "PICK TEST OFF");
	char t[160];
	snprintf(t, sizeof t, "%s / %s", HumanGround_Mode(rig.ground.mode),
			 paused ? "PAUSED" : "RUNNING");
	YMGUI_Label_SetText(rows[0], t);
	for (int side = 0; side < 2; side++)
	{
		snprintf(t, sizeof t, "%s foot: %.3f N upward", side ? "RIGHT" : "LEFT",
				 rig.ground.footLoad[side]);
		YMGUI_Label_SetText(rows[1 + side], t);
		snprintf(t, sizeof t, "%s clearance: %.1f mm", side ? "RIGHT" : "LEFT",
				 rig.ground.footClearance[side] * 1000);
		YMGUI_Label_SetText(rows[4 + side], t);
	}
	snprintf(t, sizeof t, "Total support: %.3f N", -rig.reaction.probeWrench.force.y);
	YMGUI_Label_SetText(rows[3], t);
	snprintf(t, sizeof t, "%zu contact faces / %.2f s", rig.reaction.contacts, rig.time);
	YMGUI_Label_SetText(rows[6], t);
	snprintf(t, sizeof t, "Max pressure: %.0f Pa", rig.peakPressure);
	YMGUI_Label_SetText(rows[7], t);
	snprintf(t, sizeof t, "64 x 64 / %zu unique cells", rig.count);
	YMGUI_Label_SetText(rows[8], t);
	snprintf(t, sizeof t, "Travel %.2f m / zoom %.1fx", rig.ground.travel, rig.uvZoom);
	YMGUI_Label_SetText(rows[9], t);
	snprintf(t, sizeof t, "R: 0..%.0f N | G/B: +/-%.0f N | 128=zero", 2 * rig.gain, rig.gain);
	if (rig.colorMode)
		snprintf(t, sizeof t, "%s: 0..%.0f %s", rig.colorMode == 1 ? "FORCE" : "PRESSURE",
				 rig.colorMode == 1 ? 2 * rig.gain : 2000 * rig.gain,
				 rig.colorMode == 1 ? "N" : "Pa");
	if (rig.contactView.overview == 3 && rig.contactView.showGroups)
		snprintf(t, sizeof t, "REGION COLOURS / %.1f%% neighbour edges within 2 cells",
				 rig.contactView.gridResult.withinTwoFraction * 100);
	YMGUI_Label_SetText(scale, t);
	if (rig.selectedUv >= 0)
	{
		pme_vec3 f = rig.localForces[rig.selectedUv];
		snprintf(t, sizeof t, "%s %d | Fn %.3f / Fu %+.3f / Fv %+.3f N",
				 HumanContactView_GroupName(&rig, rig.selectedUv), rig.selectedUv, f.z, f.x, f.y);
	}
	else
		snprintf(t, sizeof t, "Click skin / UV for forces | L/R focus foot | U full body");
	YMGUI_Label_SetText(info, t);
}
int main(int argc, char **argv)
{
	int smoke = argc == 2 && !strcmp(argv[1], "--smoke-test");
	int exportData = argc == 3 && !strcmp(argv[1], "--export");
	if (argc > 1 && !smoke && !exportData)
		return 2;
	YMGRE_DemoHost host;
	int ready = 0, ok = 0;
	if (!HumanSkin_Init(&rig) || !HumanGround_Enable(&rig, 0))
		goto done;
	if (exportData)
	{
		for (int i = 0; i < 420; i++)
			if (!HumanSkin_Step(&rig))
				goto done;
		ok = HumanSkin_ExportObservation(&rig, argv[2]);
		goto done;
	}
	YMGRE_DemoHost_DefaultScale(1);
	if (!YMGRE_DemoHost_Init(&host, 1280, 800, 40))
		goto done;
	ready = 1;
	view = YMGRE_DemoHost_AddTarget(&host, YMGRE_Camera_GetRenderTarget(rig.human.camera), 0, 42,
									640, 426);
	uv = YMGRE_DemoHost_AddTarget(&host, YMGRE_Camera_GetRenderTarget(rig.uvCamera), 704, 42, 512,
								  512);
	bar = YMGRE_DemoHost_AddTarget(&host, YMGRE_Camera_GetRenderTarget(rig.legend), 704, 572, 512,
								   16);
	scale = label(host.context->root, 660, 596, 620, "");
	info = label(host.context->root, 660, 624, 620, "");
	if (!view || !uv || !bar || !scale || !info)
		goto done;
	YMGUI_Image_SetScaleMode(view, GY_IMG_FIT);
	view->state |= GY_STATE_Focusable;
	view->event_cb = uv->event_cb = event;
	if (!label(host.context->root, 0, 8, 640, "HUMAN / GROUND CONTACT") ||
		!label(host.context->root, 660, 8, 620, "TOPOLOGY GRID / ONE FACE PER CELL"))
		goto done;
	for (int i = 0; i < 10; i++)
	{
		rows[i] = label(host.context->root, (i / 5) * 320, 480 + (i % 5) * 30, 320, "");
		if (!rows[i])
			goto done;
	}
	for (int i = 0; i < 8; i++)
	{
		buttons[i] = YMGUI_Creat_Button_Creat(host.context->root, 8 + i * 158, 666, 150, 32);
		if (!buttons[i])
			goto done;
		YMGUI_Button_SetText(buttons[i], names[i]);
		YMGUI_Button_SetClicked(buttons[i], clicked);
	}
	if (!label(host.context->root, 0, 716, 640,
			   "1 Walk | 2 Stand | 3 Squat | 4 Air | SPACE Pause") ||
		!label(host.context->root, 640, 716, 640, "ARROWS View | WHEEL Zoom | ESC Quit") ||
		!label(host.context->root, 0, 746, 640,
			   "L/R Foot UV | U Full map | Click skin for forces") ||
		!label(host.context->root, 640, 746, 640,
			   "V Grid/Tri | Y Regions | T RGB/F/Pa | C Range") ||
		!label(host.context->root, 0, 776, 640,
			   "Motion presets + measured ground contact forces") ||
		!label(host.context->root, 640, 776, 640, "PICK TEST: click grid / yellow X-ray triangle"))
		goto done;
	double previous = seconds(), acc = 0;
	for (int frame = 0; running && !failed; frame++)
	{
		YMGUI_SetFocus(host.context, view);
		if (smoke)
		{
			if (frame == 10)
				YMGUI_Inject_Key('4', 1);
			if (frame == 20 && (rig.ground.mode != 3 || rig.reaction.contacts))
				goto done;
			if (frame == 25)
				YMGUI_Inject_Key('2', 1);
			if (frame == 35 && (rig.ground.mode != 1 || rig.ground.footLoad[0] <= 0 ||
								rig.ground.footLoad[1] <= 0))
				goto done;
			if (frame == 40)
				YMGUI_Inject_Key('3', 1);
			if (frame == 44)
				YMGUI_Inject_Key('y', 1);
			if (frame == 45 && (rig.ground.mode != 2 || !rig.contactView.showGroups))
				goto done;
			if (frame == 46)
				YMGUI_Inject_Key('v', 1);
			if (frame == 47 && rig.contactView.overview != 1)
				goto done;
			if (frame == 48)
				YMGUI_Inject_Key('v', 1);
			if (frame == 49 && rig.contactView.overview != 3)
				goto done;
			if (frame == 50 || frame == 54 || frame == 56)
			{
				GYrect a;
				YMGUI_Obj_GetAbsArea(buttons[7], &a);
				YMGUI_Inject_Pointer(a.x + 10, a.y + 10, 1);
				YMGUI_Inject_Pointer(a.x + 10, a.y + 10, 0);
			}
			if (frame == 51 && !rig.pickTest)
				goto done;
			if (frame == 51 || frame == 57)
			{
				uint32_t c = rig.contactView.faceCells[123];
				YMGUI_Inject_Pointer(704 + (c % 64) * 8 + 4, 42 + (c / 64) * 8 + 4, 1);
				YMGUI_Inject_Pointer(704 + (c % 64) * 8 + 4, 42 + (c / 64) * 8 + 4, 0);
			}
			if (frame == 52 && rig.selectedUv != 123)
				goto done;
			if (frame == 52)
				for (uint32_t c = 0; c < HUMAN_GRID_CELLS; c++)
					if (rig.contactView.gridCells[c] == PME_TOPOLOGY_EMPTY)
					{
						YMGUI_Inject_Pointer(704 + (c % 64) * 8 + 4, 42 + (c / 64) * 8 + 4, 1);
						YMGUI_Inject_Pointer(704 + (c % 64) * 8 + 4, 42 + (c / 64) * 8 + 4, 0);
						break;
					}
			if (frame == 53 && rig.selectedUv != -1)
				goto done;
			if (frame == 55 && rig.pickTest)
				goto done;
			if (frame == 58 && (!rig.pickTest || rig.selectedUv != 123))
				goto done;
			if (frame == 60)
			{
				ok = 1;
				break;
			}
		}
		double now = seconds();
		acc += paused ? 0 : smoke ? 1. / 60 : fmin(.05, now - previous);
		previous = now;
		while (acc >= 1. / 120)
		{
			if (!HumanSkin_Step(&rig))
				goto done;
			acc -= 1. / 120;
		}
		HumanSkin_Render(&rig);
		refresh();
		YMGUI_Obj_Invalidate(view);
		YMGUI_Obj_Invalidate(uv);
		YMGUI_Obj_Invalidate(bar);
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
	HumanSkin_Destroy(&rig);
	if (smoke)
		printf("HUMAN_GROUND_WINDOW %s\n", ok ? "PASS" : "FAIL");
	return !ok;
}
