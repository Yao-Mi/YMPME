#define _POSIX_C_SOURCE 200809L
#include "rig.h"
#include "YMGRE_Coordinates_Transform.h"
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
static int running = 1, paused, failed;
static GYOBJ view, uvView, bar, rows[12], title, scale, info, buttons[22];
enum
{
	UV_X = 672,
	UV_Y = 42
};
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
	if (key >= '0' && key <= '8')
	{
		if (rig.self.enabled && !HumanSelf_Enable(&rig, 0))
			failed = 1;
		if (!HumanSkin_Region(&rig, key - '0'))
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
	case 's':
		if (!HumanSelf_Enable(&rig, !rig.self.enabled))
			failed = 1;
		break;
	case 'd':
		if (rig.self.enabled)
			rig.self.cycle = !rig.self.cycle;
		break;
	case 'g':
		if (rig.self.enabled)
			rig.self.filtered = !rig.self.filtered;
		break;
	case '+':
	case '=':
		if (rig.self.enabled)
			rig.self.cycle = 0;
		rig.indentation = fminf(.02f, rig.indentation + .001f);
		break;
	case '-':
		if (rig.self.enabled)
			rig.self.cycle = 0;
		rig.indentation = fmaxf(-.12f, rig.indentation - .001f);
		break;
	case 'o':
		if (rig.self.enabled)
			rig.self.cycle = 0;
		rig.indentation = -.12f;
		break;
	case 'f':
		rig.rub = !rig.rub;
		break;
	case 'm':
		if (rig.self.enabled)
			break;
		rig.motion = !rig.motion;
		break;
	case 't':
		rig.colorMode = (rig.colorMode + 1) % 3;
		break;
	case 'y':
		rig.contactView.showGroups = !rig.contactView.showGroups;
		break;
	case 'v':
		if (rig.self.enabled)
		{
			rig.contactView.overview = rig.contactView.overview == 3 ? 1 : 3;
			break;
		}
		rig.contactView.overview = 2;
		rig.source = !rig.source;
		rig.selectedUv = -1;
		break;
	case 'h':
		rig.heat = !rig.heat;
		break;
	case 'b':
		rig.bones = !rig.bones;
		break;
	case 'w':
		rig.wire = !rig.wire;
		break;
	case 'a':
		rig.arrows = !rig.arrows;
		break;
	case 'c':
		if (rig.self.enabled)
		{
			rig.gain = rig.gain == .02f ? .1f : rig.gain == .1f ? .005f : .02f;
			break;
		}
		rig.gain = rig.gain == 2 ? .5f : rig.gain == .5f ? 8 : 2;
		break;
	case 'z':
		YMPME_ForceDevice_TareNext(&rig.loadCell);
		rig.relative = 1;
		break;
	case 'j':
		rig.offsetU = fmaxf(-.03f, rig.offsetU - .002f);
		break;
	case 'l':
		rig.offsetU = fminf(.03f, rig.offsetU + .002f);
		break;
	case 'i':
		rig.offsetV = fmaxf(-.03f, rig.offsetV - .002f);
		break;
	case 'k':
		rig.offsetV = fminf(.03f, rig.offsetV + .002f);
		break;
	case 'p':
		if (rig.self.enabled)
			break;
		if (rig.selectedUv >= 0 && !HumanSkin_Select(&rig, rig.selectedUv))
			failed = 1;
		break;
	case 'q':
		if (rig.self.enabled)
			break;
		rig.angle = fmaxf(-45, rig.angle - 5);
		break;
	case 'e':
		if (rig.self.enabled)
			break;
		rig.angle = fminf(45, rig.angle + 5);
		break;
	case 'x':
		rig.human.axis = (rig.human.axis + 1) % 3;
		break;
	case '[':
		if (rig.self.enabled)
			break;
		if (!HumanSkin_Joint(&rig, rig.human.selected == 11 ? 62 : rig.human.selected - 1))
			failed = 1;
		break;
	case ']':
		if (rig.self.enabled)
			break;
		if (!HumanSkin_Joint(&rig, rig.human.selected == 62 ? 11 : rig.human.selected + 1))
			failed = 1;
		break;
	case '9':
	case 'n':
		if (rig.self.enabled)
			HumanContactView_Focus(&rig, key == '9' ? 1 : 2);
		break;
	case 'u':
		rig.contactView.overview = 3;
		rig.uvZoom = 1;
		rig.uvCenterU = rig.uvCenterV = .5f;
		break;
	case 'r':
	{
		rig.rub = rig.motion = rig.relative = rig.wire = rig.bones = rig.arrows = 0;
		rig.colorMode = rig.source = 0;
		rig.heat = 1;
		rig.gain = 2;
		rig.indentation = .005f;
		rig.uvZoom = 1;
		rig.uvCenterU = rig.uvCenterV = .5f;
		if (rig.self.enabled && !HumanSelf_Enable(&rig, 1))
			failed = 1;
		paused = 0;
		pme_force_device_desc d = rig.loadCell.desc;
		if (YMPME_ForceDevice_Init(&rig.loadCell, &d, 123) != PME_OK || !HumanSkin_Region(&rig, 0))
			failed = 1;
		break;
	}
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
static void viewEvent(GYOBJ obj, GYEvent e)
{
	if (e == GY_EVENT_Key)
	{
		command((int)obj->ctx->last_key);
		obj->ctx->key_handled = 1;
	}
	if (e == GY_EVENT_Wheel)
		Demo_ViewMove(&rig.human, 0, 0, -obj->ctx->wheel_y * .09f);
	if (e == GY_EVENT_Pressed)
	{
		GYrect a;
		YMGUI_Obj_GetAbsArea(obj, &a);
		int p = HumanSkin_Pick3D(&rig, (obj->ctx->point_x - a.x) * 1.5f,
								 (obj->ctx->point_y - a.y) * 640.f / 426);
		if (p >= 0 && !rig.self.enabled)
		{
			rig.region = -1;
			if (!HumanSkin_Select(&rig, p))
				failed = 1;
		}
	}
}
static void uvEvent(GYOBJ obj, GYEvent e)
{
	GYrect a;
	YMGUI_Obj_GetAbsArea(obj, &a);
	float x = obj->ctx->point_x - a.x, y = obj->ctx->point_y - a.y;
	if (e == GY_EVENT_Pressed)
		rig.selectedUv = HumanSkin_PickUV(&rig, x, y);
	if (e == GY_EVENT_Wheel)
		HumanSkin_ZoomUV(&rig, x, y, expf(obj->ctx->wheel_y * .25f));
	if (e == GY_EVENT_Key)
	{
		command((int)obj->ctx->last_key);
		obj->ctx->key_handled = 1;
	}
}
static const char *names[] = {
	"0 CHEST",		 "1 HEAD",			"2 ARM L",	   "3 ARM R",		 "4 WRIST L",
	"5 THIGH L",	 "6 THIGH R",		"7 SHIN L",	   "8 BACK",		 "+ PRESS",
	"- RELEASE",	 "F SLIDE",			"M JOINT",	   "T RGB/F/Pa",	 "V UV MAP",
	"H HEAT",		 "R RESET",			"SPACE PAUSE", "S SELF / PROBE", "D AUTO / MANUAL",
	"PICK TEST OFF", "CONTACT MARKS ON"};
static void clicked(GYOBJ obj)
{
	const char *keys = "012345678+-fmtvhr sd";
	for (int i = 0; i < 22; i++)
		if (buttons[i] == obj)
		{
			if (i == 21)
				rig.contactMarks = !rig.contactMarks;
			else if (i == 20)
				rig.pickTest = !rig.pickTest;
			else
				command(keys[i]);
		}
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
	YMGUI_Button_SetText(buttons[21], rig.contactMarks ? "CONTACT MARKS ON" : "CONTACT MARKS OFF");
	YMGUI_Button_SetText(buttons[20], rig.pickTest ? "PICK TEST ON (X-RAY)" : "PICK TEST OFF");
	char text[128];
	snprintf(text, sizeof text, "%s | %s",
			 rig.self.enabled ? HumanSelf_Phase(&rig) : HumanSkin_RegionName(rig.region),
			 paused ? "PAUSED" : "RUNNING");
	YMGUI_Label_SetText(rows[0], text);
	snprintf(text, sizeof text, "Depth %+.1f mm | probe R 75 mm", rig.indentation * 1000);
	if (rig.self.enabled)
		snprintf(text, sizeof text, "IK stop %.1f mm | error %.2f mm", rig.self.correction * 1000,
				 rig.self.ikError * 1000);
	YMGUI_Label_SetText(rows[1], text);
	snprintf(text, sizeof text, "Skin normal total: %.3f N", rig.totalNormal);
	if (rig.self.enabled)
		snprintf(text, sizeof text, "Hand %.2f N | body %.2f N", rig.self.handNormal,
				 rig.self.bodyNormal);
	YMGUI_Label_SetText(rows[2], text);
	snprintf(text, sizeof text, "Peak %.0f Pa | %zu contacts", rig.peakPressure,
			 rig.reaction.contacts);
	YMGUI_Label_SetText(rows[3], text);
	snprintf(text, sizeof text, "Probe |F| %s: %.3f N", rig.relative ? "REL" : "ABS",
			 norm(rig.loadReading.sample.wrench.force));
	if (rig.self.enabled)
		snprintf(text, sizeof text, "Hand |F| %s: %.3f N", rig.relative ? "REL" : "ABS",
				 norm(rig.loadReading.sample.wrench.force));
	YMGUI_Label_SetText(rows[4], text);
	snprintf(text, sizeof text, "Probe |T|: %.4f Nm", norm(rig.loadReading.sample.wrench.torque));
	if (rig.self.enabled)
		snprintf(text, sizeof text, "Pair residual |F|: %.2g N",
				 norm(YMPME_VecAdd(rig.self.result.sourceWrench.force,
								   rig.self.result.targetWrench.force)));
	YMGUI_Label_SetText(rows[5], text);
	snprintf(text, sizeof text, "Bone load #%d: %s", rig.loadBone,
			 rig.human.boneNames[rig.loadBone].text);
	YMGUI_Label_SetText(rows[6], text);
	snprintf(text, sizeof text, "Bone |F| %.3f N / |T| %.3f Nm",
			 norm(rig.boneLoads[rig.loadBone].force), norm(rig.boneLoads[rig.loadBone].torque));
	YMGUI_Label_SetText(rows[7], text);
	snprintf(text, sizeof text, "Joint #%d %c: %+.0f deg %s", rig.human.selected,
			 "XYZ"[rig.human.axis], rig.angle, rig.motion ? "SWING" : "HOLD");
	if (rig.self.enabled)
		snprintf(text, sizeof text, "SELF: hand / torso %s",
				 rig.self.filtered ? "IGNORED" : "COLLIDE");
	YMGUI_Label_SetText(rows[8], text);
	snprintf(text, sizeof text, "%.2f s | 120 Hz | slide %s", rig.time, rig.rub ? "ON" : "OFF");
	YMGUI_Label_SetText(rows[9], text);
	snprintf(text, sizeof text, "UV %d islands | zoom %.1fx", rig.charts, rig.uvZoom);
	YMGUI_Label_SetText(rows[10], text);
	snprintf(text, sizeof text, "RGB clip %zu | pressure cap %s", rig.rgbClipped,
			 rig.reaction.saturated ? "ON" : "OFF");
	YMGUI_Label_SetText(rows[11], text);
	snprintf(text, sizeof text, "%s | %s", rig.source ? "SOURCE UV" : "GRE AUTO UV",
			 rig.colorMode == 0	  ? "R NORMAL / G U / B V"
			 : rig.colorMode == 1 ? "FORCE MAGNITUDE"
								  : "NORMAL PRESSURE");
	if (rig.contactView.overview == 1)
		snprintf(text, sizeof text, "FULL BODY / 3584 EQUAL SENSOR TRIANGLES");
	if (rig.contactView.overview == 3)
		snprintf(text, sizeof text, "TOPOLOGY GRID / ONE FACE PER CELL / V TRIANGLES");
	YMGUI_Label_SetText(title, text);
	if (rig.colorMode == 0)
		snprintf(text, sizeof text, "R: 0..%.3f N | G/B: +/-%.3f N | 128=zero", 2 * rig.gain,
				 rig.gain);
	else
		snprintf(text, sizeof text, "0 ---------------------------- %.0f %s",
				 rig.colorMode == 1 ? 2 * rig.gain : 2000 * rig.gain,
				 rig.colorMode == 1 ? "N" : "Pa");
	YMGUI_Label_SetText(scale, text);
	if (rig.selectedUv >= 0)
	{
		pme_vec3 f = rig.localForces[rig.selectedUv];
		snprintf(text, sizeof text, "%d: Fn %.3f / Fu %+.3f / Fv %+.3f N", rig.selectedUv, f.z, f.x,
				 f.y);
	}
	else
		snprintf(text, sizeof text, "%s",
				 rig.selectedUv == -2 ? "Overlapping UV: no unique face"
									  : "CLICK UV readout | WHEEL UV zoom | U overview");
	YMGUI_Label_SetText(info, text);
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
		headless = argc == 2 && !strcmp(argv[1], "--headless"), ok = 0, ready = 0;
	YMGRE_DemoHost host;
	unsigned progress = 0;
	if (argc > 1 && !smoke && !headless)
		return 2;
	if (!HumanSkin_Init(&rig))
		goto done;
	rig.rub = 1;
	if (!smoke && !HumanSelf_Enable(&rig, 1))
		goto done;
	if (headless)
	{
		for (int i = 0; i < 420; i++)
			if (!HumanSkin_Step(&rig))
				goto done;
		HumanSkin_Render(&rig);
		ok = Demo_Save(&rig.human, "human_skin.ppm");
		goto done;
	}
	YMGRE_DemoHost_DefaultScale(1);
	if (!YMGRE_DemoHost_Init(&host, 1280, 800, 40))
		goto done;
	ready = 1;
	view = YMGRE_DemoHost_AddTarget(&host, YMGRE_Camera_GetRenderTarget(rig.human.camera), 0, 42,
									640, 426);
	uvView = YMGRE_DemoHost_AddTarget(&host, YMGRE_Camera_GetRenderTarget(rig.uvCamera), UV_X, UV_Y,
									  UV_VIEW, UV_VIEW);
	bar = YMGRE_DemoHost_AddTarget(&host, YMGRE_Camera_GetRenderTarget(rig.legend), UV_X, 572,
								   UV_VIEW, 16);
	title = label(host.context->root, UV_X, 8, 600, "");
	scale = label(host.context->root, UV_X, 596, 600, "");
	info = label(host.context->root, UV_X, 622, 600, "");
	if (!view || !uvView || !bar || !title || !scale || !info)
		goto done;
	YMGUI_Image_SetScaleMode(view, GY_IMG_FIT);
	view->state |= GY_STATE_Focusable;
	view->event_cb = viewEvent;
	uvView->event_cb = uvEvent;
	if (!label(host.context->root, 0, 8, 640, "HUMAN SELF TOUCH / HAND + BODY RGB FEEDBACK"))
		goto done;
	for (int i = 0; i < 12; i++)
	{
		rows[i] = label(host.context->root, (i / 6) * 320, 480 + (i % 6) * 25, 320, "");
		if (!rows[i])
			goto done;
	}
	for (int i = 0; i < 22; i++)
	{
		buttons[i] = YMGUI_Creat_Button_Creat(host.context->root,
											  i < 18	? 4 + (i % 9) * 142
											  : i == 21 ? 800
														: 1040,
											  i < 18	? 654 + (i / 9) * 34
											  : i == 21 ? 777
														: 727 + (i - 18) * 25,
											  i < 18 ? 134 : 230, i < 18 ? 28 : 22);
		if (!buttons[i])
			goto done;
		YMGUI_Button_SetText(buttons[i], names[i]);
		YMGUI_Button_SetClicked(buttons[i], clicked);
	}
	if (!label(host.context->root, 0, 727, 1030,
			   "J/L I/K Slide | O Lift hand | F Slide | G Filter hand/body") ||
		!label(host.context->root, 0, 752, 1030,
			   "9 Hand | N Body | U Full | Y Regions | V Grid/Tri | B Bones | W Wire") ||
		!label(host.context->root, 0, 777, 790,
			   "WHITE Contact (X-ray) | ARROWS View | WHEEL Zoom | C Range"))
		goto done;
	double previous = seconds(), accumulator = 0;
	for (unsigned frame = 0; running && !failed; frame++)
	{
		progress = frame;
		YMGUI_SetFocus(host.context, view);
		if (smoke)
		{
			if (frame == 5)
				clickButton(1);
			if (frame == 6 && rig.region != 1)
				goto done;
			if (frame == 10)
				clickButton(9);
			if (frame == 11 && rig.indentation <= .005f)
				goto done;
			if (frame == 15)
				clickButton(12);
			if (frame == 16 && !rig.motion)
				goto done;
			if (frame == 20)
				clickButton(13);
			if (frame == 21 && rig.colorMode != 1)
				goto done;
			if (frame == 25)
				clickButton(14);
			if (frame == 26 && !rig.source)
				goto done;
			if (frame == 30)
				YMGUI_Inject_Key('z', 1);
			if (frame == 31 && !rig.relative)
				goto done;
			if (frame == 35)
				clickButton(16);
			if (frame == 36 &&
				(rig.region != 0 || rig.colorMode != 0 || rig.source || rig.relative))
				goto done;
			if (frame == 42)
			{
				pme_vec3 pos = rig.patches[rig.probePatch].position;
				gre_fvector4d p = {pos.x * 10, pos.y * 10, pos.z * 10, 1}, cam;
				YMGRE_Point_WorldToCamera(&p, &cam, &rig.human.camera->move.TMat);
				YMGRE_Point_CameraToViewPlane(&cam, rig.human.camera->perspectPlane.Dis);
				YMGRE_Point_ViewPlaneToWindows(&cam, rig.human.camera);
				int x = (int)lroundf(cam.x / 1.5f), y = 42 + (int)lroundf(cam.y * 426 / 640);
				YMGUI_Inject_Pointer(x, y, 1);
				YMGUI_Inject_Pointer(x, y, 0);
				if (rig.region != -1)
					goto done;
			}
			if (frame == 45)
			{
				int found = -1;
				for (int y = 0; y < UV_VIEW && found < 0; y++)
					for (int x = 0; x < UV_VIEW; x++)
					{
						int p = HumanSkin_PickUV(&rig, x, y);
						if (p >= 0 && rig.taxels[p].normalForce > 0)
						{
							found = y * UV_VIEW + x;
							break;
						}
					}
				if (found < 0)
					goto done;
				YMGUI_Inject_Pointer(UV_X + found % UV_VIEW, UV_Y + found / UV_VIEW, 1);
				YMGUI_Inject_Pointer(UV_X + found % UV_VIEW, UV_Y + found / UV_VIEW, 0);
				if (rig.selectedUv < 0)
					goto done;
			}
			if (frame == 50)
				YMGUI_Inject_Key('o', 1);
			if (frame == 55 && rig.reaction.contacts)
				goto done;
			if (frame == 60)
			{
				if (!rig.touch.stamp.valid || rig.totalNormal != 0)
					goto done;
				clickButton(18);
			}
			if (frame == 61 && !rig.self.enabled)
				goto done;
			if (frame == 65)
				clickButton(19);
			if (frame == 66 && rig.self.cycle)
				goto done;
			if (frame == 70)
				YMGUI_Inject_Key('g', 1);
			if (frame == 71 && !rig.self.filtered)
				goto done;
			if (frame == 75)
				clickButton(18);
			if (frame == 76 && rig.self.enabled)
				goto done;
			if (frame == 77)
				clickButton(20);
			if (frame == 78 && !rig.pickTest)
				goto done;
			if (frame == 78)
				clickButton(20);
			if (frame == 79 && rig.pickTest)
				goto done;
			if (frame == 80)
				clickButton(21);
			if (frame == 81 && rig.contactMarks)
				goto done;
			if (frame == 82)
				clickButton(21);
			if (frame == 83 && !rig.contactMarks)
				goto done;
			if (frame == 84)
			{
				ok = 1;
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
			while (accumulator >= 1. / 120)
			{
				if (!HumanSkin_Step(&rig))
					goto done;
				accumulator -= 1. / 120;
			}
		}
		HumanSkin_Render(&rig);
		refresh();
		YMGUI_Obj_Invalidate(view);
		YMGUI_Obj_Invalidate(uvView);
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
		printf("HUMAN_SKIN_WINDOW %s (frame %u)\n", ok ? "PASS" : "FAIL", progress);
	return !ok;
}
