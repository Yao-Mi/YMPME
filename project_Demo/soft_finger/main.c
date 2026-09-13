#include "render.h"
#include "demo_host.h"
#include "YMGUI_Label.h"
#include "YMGUI_Invalidate.h"
#include <stdio.h>
#include <string.h>
enum
{
	PAUSE = 1,
	STEP = 2,
	RESET = 4,
	AUTO = 8,
	HARD = 16,
	SLOW = 32,
	CAMERA = 64,
	WIRE = 128,
	FRICTION = 256,
	QUIT = 512,
	BALL = 1024
};
enum
{
	CLOSE = 1,
	OPEN = 2,
	UP = 4,
	DOWN = 8
};
static const YMGRE_DemoKeyBinding keys[] = {{YMGRE_KEY_SPACE, 0, PAUSE, 0},
											{'n', 0, STEP, 0},
											{'r', 0, RESET, 0},
											{'t', 0, AUTO, 0},
											{'k', 0, HARD, 0},
											{'b', 0, BALL, 0},
											{'s', 0, SLOW, 0},
											{'c', 0, CAMERA, 0},
											{'v', 0, WIRE, 0},
											{'f', 0, FRICTION, 0},
											{YMGRE_KEY_ESCAPE, 0, QUIT, 0},
											{'1', 0, 0, 1},
											{'2', 0, 0, 2},
											{'3', 0, 0, 3},
											{'a', CLOSE, 0, 0},
											{'d', OPEN, 0, 0},
											{YMGRE_KEY_UP, UP, 0, 0},
											{YMGRE_KEY_DOWN, DOWN, 0, 0}};
static GYOBJ label(GYOBJ root, int x, int y, int width, const char *text)
{
	GYOBJ l = YMGUI_Creat_Label_Creat(root, x, y, width, 24);
	if (l)
	{
		YMGUI_Label_SetText(l, text);
		YMGUI_Label_SetTextColor(l, GY_ARGB(255, 224, 234, 244));
	}
	return l;
}
static int step(soft_finger *s, unsigned held)
{
	if (!s->grasp.automatic)
	{
		s->grasp.targetAngle = YMPME_Clamp(
			s->grasp.targetAngle + (((held & CLOSE) != 0) - ((held & OPEN) != 0)) * .5f * GRASP_DT,
			-.55f, .55f);
		s->grasp.targetHeight = YMPME_Clamp(
			s->grasp.targetHeight + (((held & UP) != 0) - ((held & DOWN) != 0)) * .25f * GRASP_DT,
			0, .75f);
	}
	return SoftFinger_Step(s);
}
static int tap(YMGRE_DemoHost *h, int k)
{
	return YMGRE_DemoHost_InjectKey(h, k, 1) && YMGRE_DemoHost_InjectKey(h, k, 0);
}
int main(int argc, char **argv)
{
	static soft_finger s;
	static soft_view view;
	YMGRE_DemoHost host;
	GYOBJ targets[5], rows[9];
	int ready = 0, rc = 1, paused = 0, slow = 0, camera = 0, wire = 1, verified = 0;
	int smoke = argc == 2 && !strcmp(argv[1], "--smoke-test");
	int snapshot = argc == 2 && !strcmp(argv[1], "--snapshot");
	if (argc > 1 && !smoke && !snapshot)
	{
		fprintf(stderr, "Usage: %s [--smoke-test|--snapshot]\n", argv[0]);
		return 1;
	}
	if (!SoftFinger_Init(&s, 0, 0) || !SoftView_Init(&view))
		goto done;
	if (snapshot)
	{
		for (int i = 0; i < 240 * 6; ++i)
			if (!step(&s, 0))
				goto done;
		paused = 1;
	}
	YMGRE_DemoHost_DefaultScale(1);
	if (!YMGRE_DemoHost_Init(&host, 1200, 840, 40))
	{
		fprintf(stderr, "GRE: %s\n", YMGRE_DemoHost_LastError());
		goto done;
	}
	ready = 1;
	YMGRE_DemoHost_SetTitle(&host, "YMPME + GRE | Extra-soft ball | squeeze / bulge / recovery");
	targets[0] = YMGRE_DemoHost_AddTarget(&host, YMGRE_Camera_GetRenderTarget(view.scene.camera), 0,
										  0, 840, 640);
	for (int side = 0; side < 2; ++side)
	{
		targets[side + 1] =
			YMGRE_DemoHost_AddTarget(&host, view.scene.maps[side], 858 + 168 * side, 64, 144, 288);
		targets[side + 3] = YMGRE_DemoHost_AddTarget(&host, view.indentation[side],
													 858 + 168 * side, 432, 144, 288);
	}
	for (int i = 0; i < 5; ++i)
		if (!targets[i])
			goto done;
	const char *names[] = {"PRESSURE: 0 .. 2 kPa",	  "LEFT  8 x 16", "RIGHT 8 x 16",
						   "INDENTATION: 0 .. 10 mm", "LEFT  8 x 16", "RIGHT 8 x 16"};
	for (int i = 0; i < 6; ++i)
		if (!label(host.context->root, i % 3 == 2 ? 1026 : 858, (i / 3) * 368 + (i % 3 ? 36 : 8),
				   i % 3 ? 155 : 330, names[i]))
			goto done;
	for (int i = 0; i < 5; ++i)
	{
		rows[i] = label(host.context->root, 12, 646 + i * 27, 825, "");
		if (!rows[i])
			goto done;
	}
	for (int i = 0; i < 4; ++i)
	{
		rows[i + 5] = label(host.context->root, 858 + (i % 2) * 168, i < 2 ? 355 : 722, 164, "");
		if (!rows[i + 5])
			goto done;
	}
	const char *help[] = {"1/2/3 Shape | K Pads | B Ball soft/firm (reset)",
						  "T Auto/manual | A/D Close/open | UP/DOWN Lift",
						  "SPACE Pause | N Step | S Slow 0.1x | R Reset",
						  "C Camera | V Object wire | F Friction | ESC Exit"};
	for (int i = 0; i < 4; ++i)
		if (!label(host.context->root, i % 2 ? 610 : 12, 782 + 27 * (i / 2), 588, help[i]))
			goto done;
	if (!label(host.context->root, 858, 753, 336, "3D deformation: true scale") ||
		!YMGRE_DemoHost_BindKeys(&host, keys, sizeof(keys) / sizeof(keys[0])))
		goto done;
	double previous = YMGRE_DemoHost_Time(), accumulator = 0;
	uint64_t saved = 0;
	for (int frame = 0;; ++frame)
	{
		double now = YMGRE_DemoHost_Time(), elapsed = smoke ? 1. / 60 : now - previous;
		previous = now;
		if (smoke)
		{
			if (frame == 5)
			{
				saved = s.grasp.steps;
				if (!tap(&host, YMGRE_KEY_SPACE))
					goto done;
			}
			if (frame == 8)
			{
				if (s.grasp.steps != saved || !tap(&host, 'n'))
					goto done;
			}
			if (frame == 9 && s.grasp.steps != saved + 1)
				goto done;
			if (frame == 10 && !tap(&host, YMGRE_KEY_SPACE))
				goto done;
			if (frame == 40 && !tap(&host, 'c'))
				goto done;
			if (frame == 45 && !tap(&host, 'v'))
				goto done;
			if (frame == 365)
			{
				if (s.grasp.bodies[3].position.y < .75f || s.result[0].maxCompression < .001f ||
					camera != 1 || wire || YMPME_SoftBall_Scale(&s.ball).x > .7f)
					goto done;
				verified = 1;
			}
			if (frame == 550)
			{
				if (s.result[0].maxCompression > 1e-5f || s.grasp.bodies[3].position.y > .25f ||
					!tap(&host, 'b'))
					goto done;
			}
			if (frame == 552 && (!s.ballFirm || s.ball.stiffness != 600.f || !tap(&host, 'k')))
				goto done;
			if (frame == 554 && (!s.hard || !tap(&host, '2')))
				goto done;
			if (frame == 556 && (s.grasp.shape != 1 || !tap(&host, 't')))
				goto done;
			if (frame == 557 && (!YMGRE_DemoHost_InjectKey(&host, 'a', 1) ||
								 !YMGRE_DemoHost_InjectKey(&host, YMGRE_KEY_UP, 1)))
				goto done;
			if (frame == 561)
			{
				if (s.grasp.automatic || s.grasp.targetHeight <= 0 || !tap(&host, YMGRE_KEY_ESCAPE))
					goto done;
			}
		}
		unsigned actions = YMGRE_DemoHost_TakeActions(&host), held = YMGRE_DemoHost_HeldKeys(&host);
		int value = YMGRE_DemoHost_TakeValue(&host);
		if (actions & QUIT)
		{
			rc = smoke ? !verified : 0;
			break;
		}
		if (value || (actions & (RESET | HARD | BALL)) ||
			(s.grasp.automatic && s.grasp.world.time > 11))
		{
			int shape = value ? value - 1 : s.grasp.shape, hard = s.hard ^ !!(actions & HARD);
			int ballFirm = s.ballFirm ^ !!(actions & BALL);
			if (!SoftFinger_Init(&s, shape, hard))
				goto done;
			s.ballFirm = ballFirm;
			s.ball.stiffness = ballFirm ? 600.f : 40.f;
			paused = 0;
			accumulator = 0;
			YMGRE_DemoHost_ClearInput(&host);
			held = 0;
		}
		if (actions & PAUSE)
		{
			paused = !paused;
			accumulator = 0;
		}
		if (actions & AUTO)
			s.grasp.automatic = !s.grasp.automatic;
		if (actions & SLOW)
			slow = !slow;
		if (actions & CAMERA)
			camera = (camera + 1) % 3;
		if (actions & WIRE)
			wire = !wire;
		if (actions & FRICTION)
			s.grasp.friction = s.grasp.friction > 0 ? 0 : .85f;
		if (paused && (actions & STEP) && !step(&s, held))
			goto done;
		if (!paused)
		{
			accumulator += fmin(elapsed, .1) * (slow ? .1 : 1);
			while (accumulator >= 1. / 240)
			{
				if (!step(&s, held))
					goto done;
				accumulator -= 1. / 240;
			}
		}
		SoftView_Draw(&view, &s, camera, wire);
		for (int i = 0; i < 5; ++i)
			YMGUI_Obj_Invalidate(targets[i]);
		char text[64];
		snprintf(text, sizeof(text), "%s | %s | %s | t %.2f s | speed %.1fx",
				 s.grasp.automatic ? "AUTO" : "MANUAL", s.hard ? "FIRM" : "SOFT",
				 paused ? "PAUSED" : "RUN", s.grasp.world.time, slow ? .1 : 1);
		YMGUI_Label_SetText(rows[0], text);
		snprintf(text, sizeof(text), "Object %.3f m | grip Fy %.3f / mg 2.943 N",
				 s.grasp.bodies[3].position.y, s.grasp.gripForce.y);
		YMGUI_Label_SetText(rows[1], text);
		snprintf(text, sizeof(text), "k %.0f Pa/m | mu %.2f | sample contacts %zu",
				 s.material.stiffness, s.material.friction,
				 s.result[0].contacts + s.result[1].contacts);
		YMGUI_Label_SetText(rows[2], text);
		snprintf(text, sizeof(text), "Bottomed samples %zu | elastic energy %.5f J",
				 s.result[0].bottomedOut + s.result[1].bottomedOut,
				 s.result[0].elasticEnergy + s.result[1].elasticEnergy);
		YMGUI_Label_SetText(rows[3], text);
		if (s.grasp.shape == 0)
		{
			pme_vec3 scale = YMPME_SoftBall_Scale(&s.ball);
			snprintf(text, sizeof(text), "Ball %s | X %.0f%% Y/Z %.0f%% | volume 100%%",
					 s.ballFirm ? "FIRM" : "EXTRA SOFT", 100 * scale.x, 100 * scale.y);
			YMGUI_Label_SetText(rows[4], text);
		}
		else
			YMGUI_Label_SetText(rows[4], "Rigid object; soft pads; fixed sample areas.");
		for (int side = 0; side < 2; ++side)
		{
			snprintf(text, sizeof(text), "Fn %.2f N", Grasp_NormalForce(&s.grasp, side));
			YMGUI_Label_SetText(rows[5 + side], text);
			snprintf(text, sizeof(text), "max %.2f mm", s.result[side].maxCompression * 1000);
			YMGUI_Label_SetText(rows[7 + side], text);
		}
		if (!YMGRE_DemoHost_Step(&host, smoke ? 0 : 60))
		{
			rc = smoke ? 1 : 0;
			break;
		}
	}
done:
	if (ready)
		YMGRE_DemoHost_Destroy(&host);
	SoftView_Destroy(&view);
	if (smoke)
		printf("SOFT_FINGER_WINDOW %s\n", rc ? "FAIL" : "PASS");
	else if (rc)
		fprintf(stderr, "Soft finger initialization or simulation failed\n");
	return rc;
}
