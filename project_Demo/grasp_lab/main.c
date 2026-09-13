#include "scene.h"
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
	FRICTION = 16,
	MOTOR = 32,
	CAMERA = 64,
	WIRE = 128,
	PRESSURE = 256,
	QUIT = 512
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
											{YMGRE_KEY_HOME, 0, RESET, 0},
											{'t', 0, AUTO, 0},
											{'f', 0, FRICTION, 0},
											{'m', 0, MOTOR, 0},
											{'c', 0, CAMERA, 0},
											{'v', 0, WIRE, 0},
											{'p', 0, PRESSURE, 0},
											{YMGRE_KEY_ESCAPE, 0, QUIT, 0},
											{'1', 0, 0, 1},
											{'2', 0, 0, 2},
											{'3', 0, 0, 3},
											{'4', 0, 0, 4},
											{'a', CLOSE, 0, 0},
											{'d', OPEN, 0, 0},
											{YMGRE_KEY_UP, UP, 0, 0},
											{YMGRE_KEY_DOWN, DOWN, 0, 0}};
static GYOBJ label(GYOBJ parent, int x, int y, int width, const char *text)
{
	GYOBJ p = YMGUI_Creat_Label_Creat(parent, x, y, width, 24);
	if (p)
	{
		YMGUI_Label_SetText(p, text);
		YMGUI_Label_SetTextColor(p, GY_ARGB(255, 223, 233, 242));
	}
	return p;
}
static int tap(YMGRE_DemoHost *h, int k)
{
	return YMGRE_DemoHost_InjectKey(h, k, 1) && YMGRE_DemoHost_InjectKey(h, k, 0);
}
static uint64_t hash(GRE_RenderTarget t)
{
	uint64_t h = UINT64_C(14695981039346656037);
	const unsigned char *p = (const unsigned char *)t->data;
	for (size_t i = 0; i < (size_t)t->width * t->height * sizeof(*t->data); ++i)
		h = (h ^ p[i]) * UINT64_C(1099511628211);
	return h;
}
static int physics(grasp_lab *s, unsigned held)
{
	if (!s->automatic)
	{
		s->targetAngle = YMPME_Clamp(
			s->targetAngle + (((held & CLOSE) != 0) - ((held & OPEN) != 0)) * .5f * GRASP_DT, -.55f,
			.55f);
		s->targetHeight = YMPME_Clamp(
			s->targetHeight + (((held & UP) != 0) - ((held & DOWN) != 0)) * .25f * GRASP_DT, 0,
			.75f);
	}
	return Grasp_Step(s);
}
int main(int argc, char **argv)
{
	static grasp_lab state;
	static grasp_scene scene;
	YMGRE_DemoHost host;
	GYOBJ targets[3], rows[12];
	int ready = 0, result = 1, paused = 0, view = 0, wire = 0, pressure = 0;
	int smoke = argc == 2 && !strcmp(argv[1], "--smoke-test"), verified = 0;
	int initialShape = argc == 2 && !strcmp(argv[1], "--box")	   ? 1
					   : argc == 2 && !strcmp(argv[1], "--convex") ? 2
																   : 0;
	uint64_t savedSteps = 0, first = 0;
	float manualAngle = 0, manualHeight = 0;
	double accumulator = 0, previous;
	if (argc > 1 && !smoke && !initialShape)
	{
		fprintf(stderr, "Usage: %s [--smoke-test|--box|--convex]\n", argv[0]);
		return 1;
	}
	if (!Grasp_InitShape(&state, 0, initialShape) || !GraspScene_Init(&scene))
		goto done;
	YMGRE_DemoHost_DefaultScale(1);
	if (!YMGRE_DemoHost_Init(&host, 1200, 820, 40))
	{
		fprintf(stderr, "GRE: %s\n", YMGRE_DemoHost_LastError());
		goto done;
	}
	ready = 1;
	YMGRE_DemoHost_SetTitle(&host,
							"YMPME + GRE | Grasp lab - sphere / box / convex | finger tactile");
	targets[0] =
		YMGRE_DemoHost_AddTarget(&host, YMGRE_Camera_GetRenderTarget(scene.camera), 0, 0, 840, 640);
	targets[1] = YMGRE_DemoHost_AddTarget(&host, scene.maps[0], 858, 60, 144, 288);
	targets[2] = YMGRE_DemoHost_AddTarget(&host, scene.maps[1], 1026, 60, 144, 288);
	for (int i = 0; i < 3; ++i)
		if (!targets[i])
			goto done;
	if (!label(host.context->root, 858, 5, 330, "SOLVED CONTACT -> TACTILE") ||
		!label(host.context->root, 858, 32, 160, "LEFT  8 x 16") ||
		!label(host.context->root, 1026, 32, 160, "RIGHT 8 x 16") ||
		!label(host.context->root, 858, 382, 330, "OBJECT CONTACT LOAD"))
		goto done;
	for (int i = 0; i < 3; ++i)
	{
		rows[i] = label(host.context->root, 12, 650 + 27 * i, 810, "");
		if (!rows[i])
			goto done;
	}
	rows[3] = label(host.context->root, 858, 352, 160, "");
	rows[4] = label(host.context->root, 1026, 352, 160, "");
	rows[5] = label(host.context->root, 858, 410, 335, "");
	rows[6] = label(host.context->root, 858, 439, 335, "");
	rows[7] = label(host.context->root, 858, 468, 335, "");
	rows[8] = label(host.context->root, 858, 555, 335, "");
	rows[9] = label(host.context->root, 858, 584, 335, "");
	rows[10] = label(host.context->root, 858, 497, 335, "");
	rows[11] = label(host.context->root, 858, 526, 335, "");
	for (int i = 3; i < 12; ++i)
		if (!rows[i])
			goto done;
	const char *help[6] = {"1 Push | 2 Sphere | 3 Box | 4 Convex | T Auto",
						   "A/D Close/open | UP/DOWN Lift/lower (manual)",
						   "F Friction on/off | M Motors on/off | R Reset",
						   "C Camera | V Object wireframe | P RGB/pressure",
						   "SPACE Pause | N Single physics step | ESC Exit",
						   "Green arrows: object force. Orange: reaction."};
	for (int i = 0; i < 6; ++i)
		if (!label(host.context->root, i % 2 ? 610 : 12, 733 + 27 * (i / 2), 588, help[i]))
			goto done;
	if (!YMGRE_DemoHost_BindKeys(&host, keys, sizeof keys / sizeof keys[0]))
		goto done;
	previous = YMGRE_DemoHost_Time();
	for (int frame = 0;; ++frame)
	{
		double now = YMGRE_DemoHost_Time(), elapsed = smoke ? 1. / 60 : now - previous;
		previous = now;
		if (smoke)
		{
			if (frame == 10)
			{
				savedSteps = state.steps;
				if (!tap(&host, YMGRE_KEY_SPACE))
					goto done;
			}
			if (frame == 13)
			{
				if (state.steps != savedSteps || !tap(&host, 'n'))
					goto done;
			}
			if (frame == 14 && state.steps != savedSteps + 1)
				goto done;
			if (frame == 16 && !tap(&host, YMGRE_KEY_SPACE))
				goto done;
			if (frame == 100 && !tap(&host, 'c'))
				goto done;
			if (frame == 110 && !tap(&host, 'v'))
				goto done;
			if (frame == 120 && !tap(&host, 'p'))
				goto done;
			if (frame == 340)
			{
				if (state.bodies[3].position.y < .7f || Grasp_NormalForce(&state, 0) < 1 ||
					Grasp_NormalForce(&state, 1) < 1 || view != 1 || !wire || !pressure ||
					first == hash(YMGRE_Camera_GetRenderTarget(scene.camera)))
					goto done;
				verified = 1;
				if (!tap(&host, 't'))
					goto done;
				manualAngle = state.targetAngle;
				manualHeight = state.targetHeight;
			}
			if (frame == 341 && (!YMGRE_DemoHost_InjectKey(&host, 'a', 1) ||
								 !YMGRE_DemoHost_InjectKey(&host, YMGRE_KEY_UP, 1)))
				goto done;
			if (frame == 350)
			{
				if (state.targetAngle <= manualAngle || state.targetHeight <= manualHeight ||
					state.automatic)
					goto done;
				if (!YMGRE_DemoHost_InjectKey(&host, 'a', 0) ||
					!YMGRE_DemoHost_InjectKey(&host, YMGRE_KEY_UP, 0))
					goto done;
			}
			if (frame == 355 && !tap(&host, 'f'))
				goto done;
			if (frame == 356 && !tap(&host, 'm'))
				goto done;
			if (frame == 360 && (state.friction != 0 || state.motors))
				goto done;
			if (frame == 365 && !tap(&host, 'r'))
				goto done;
			if (frame == 370 && !tap(&host, '1'))
				goto done;
			if (frame == 375 && state.mode != 1)
				goto done;
			if (frame == 380 && !tap(&host, '2'))
				goto done;
			if (frame == 400 && !tap(&host, '3'))
				goto done;
			if (frame == 765 && (state.shape != 1 || state.bodies[3].position.y < .7f))
				goto done;
			if (frame == 770 && !tap(&host, '4'))
				goto done;
			if (frame == 1135 && (state.shape != 2 || state.bodies[3].position.y < .7f))
				goto done;
			if (frame == 1140 && !tap(&host, YMGRE_KEY_ESCAPE))
				goto done;
		}
		unsigned actions = YMGRE_DemoHost_TakeActions(&host), held = YMGRE_DemoHost_HeldKeys(&host);
		int value = YMGRE_DemoHost_TakeValue(&host);
		if (actions & QUIT)
		{
			result = smoke ? !(verified && state.shape == 2 && state.mode == 0 && state.automatic &&
							   state.motors && state.friction > 0)
						   : 0;
			break;
		}
		if ((actions & RESET) || value || (state.automatic && !state.mode && state.world.time > 11))
		{
			int mode = value ? value == 1 : state.mode;
			int shape = value ? (value == 1 ? 0 : value - 2) : state.shape;
			int repeat = !(actions & RESET) && !value;
			float friction = state.friction;
			int motors = state.motors;
			if (!Grasp_InitShape(&state, mode, shape))
				goto done;
			if (repeat)
			{
				state.friction = friction;
				state.motors = motors;
			}
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
			state.automatic = !state.automatic;
		if (actions & FRICTION)
			state.friction = state.friction > 0 ? 0 : .85f;
		if (actions & MOTOR)
			state.motors = !state.motors;
		if (actions & CAMERA)
			view = (view + 1) % 3;
		if (actions & WIRE)
			wire = !wire;
		if (actions & PRESSURE)
			pressure = !pressure;
		if (paused && (actions & STEP) && !physics(&state, held))
			goto done;
		if (!paused)
		{
			accumulator += fmin(elapsed, .1);
			while (accumulator >= 1. / 240)
			{
				if (!physics(&state, held))
					goto done;
				accumulator -= 1. / 240;
			}
		}
		GraspScene_Render(&scene, &state, view, wire, pressure);
		for (int i = 0; i < 3; ++i)
			YMGUI_Obj_Invalidate(targets[i]);
		if (!frame)
			first = hash(YMGRE_Camera_GetRenderTarget(scene.camera));
		char text[64];
		snprintf(text, sizeof text, "%s | %s | %s | t %.2f s | 240 Hz",
				 state.mode			? "PUSH"
				 : state.shape == 0 ? "SPHERE GRIP"
				 : state.shape == 1 ? "BOX GRIP"
									: "CONVEX GRIP",
				 state.automatic ? "AUTO" : "MANUAL", paused ? "PAUSED" : "RUN", state.world.time);
		YMGUI_Label_SetText(rows[0], text);
		snprintf(text, sizeof text, "Object y %.3f m | vx %+.3f m/s | spin z %+.3f rad/s",
				 state.bodies[3].position.y, state.bodies[3].velocity.x,
				 state.bodies[3].angularVelocity.z);
		YMGUI_Label_SetText(rows[1], text);
		snprintf(text, sizeof text, "mu %.2f | motors %s | torque %+.2f / %+.2f Nm", state.friction,
				 state.motors ? "ON" : "OFF", state.joints[1].feedback.motorEffort,
				 state.joints[2].feedback.motorEffort);
		YMGUI_Label_SetText(rows[2], text);
		for (int i = 0; i < 2; ++i)
		{
			snprintf(text, sizeof text, "Fn %.3f N", Grasp_NormalForce(&state, i));
			YMGUI_Label_SetText(rows[3 + i], text);
		}
		snprintf(text, sizeof text, "Fx %+.3f Fy %+.3f Fz %+.3f N", state.objectContact.force.x,
				 state.objectContact.force.y, state.objectContact.force.z);
		YMGUI_Label_SetText(rows[5], text);
		snprintf(text, sizeof text, "Lift Fy %.3f / mg 2.943 N", state.gripForce.y);
		YMGUI_Label_SetText(rows[6], text);
		snprintf(text, sizeof text, "Ground Fy %.3f N", state.groundForce.y);
		YMGUI_Label_SetText(rows[7], text);
		snprintf(text, sizeof text, "Torque z %+.4f Nm", state.objectContact.torque.z);
		YMGUI_Label_SetText(rows[10], text);
		snprintf(text, sizeof text, "Contact points %zu | object 0.30 kg",
				 state.world.contactCount);
		YMGUI_Label_SetText(rows[11], text);
		YMGUI_Label_SetText(rows[8], pressure ? "Pressure: 0..15 kPa (cell mean)"
											  : "RGB: Fn / shear U / shear V");
		YMGUI_Label_SetText(rows[9], pressure ? "Fixed cell area; point-load model"
											  : "R 0..4N | G/B +/-2N (zero 128)");
		if (!YMGRE_DemoHost_Step(&host, smoke ? 0 : 60))
		{
			result = smoke ? 1 : 0;
			break;
		}
	}
done:
	if (ready)
		YMGRE_DemoHost_Destroy(&host);
	GraspScene_Destroy(&scene);
	if (smoke)
		printf("GRASP_WINDOW %s (GRE input, physics, tactile, frames)\n", result ? "FAIL" : "PASS");
	if (result && !smoke)
		fprintf(stderr, "Grasp lab initialization or simulation failed\n");
	return result;
}
