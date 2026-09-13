#include "flight.h"
#include "YMGRE_Camera.h"
#include "YMGUI_Label.h"
#include "YMGUI_Invalidate.h"
#include "demo_host.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
enum
{
	PAUSE = 1,
	STEP = 2,
	RESET = 4,
	AUTO = 8,
	VIEW = 16,
	SWITCH = 32,
	POWER = 64,
	WIND = 128,
	QUIT = 256,
	CONTROL = 512
};
enum
{
	FORWARD = 1,
	BACK = 2,
	LEFT = 4,
	RIGHT = 8,
	UP = 16,
	DOWN = 32,
	YAWLEFT = 64,
	YAWRIGHT = 128
};

typedef struct simulation_
{
	owl_flight owl;
	int paused, automatic, power, wind, view, wire, bones;
	double accumulator;
	unsigned steps;
	pme_aircraft_command command;
} simulation;
static const YMGRE_DemoKeyBinding keyBindings[] = {
	{'w', FORWARD, 0, 0},
	{'s', BACK, 0, 0},
	{'a', LEFT, 0, 0},
	{'d', RIGHT, 0, 0},
	{YMGRE_KEY_UP, UP, 0, 0},
	{YMGRE_KEY_DOWN, DOWN, 0, 0},
	{'q', YAWLEFT, 0, 0},
	{'e', YAWRIGHT, 0, 0},
	{'b', 0, CONTROL, 0},
	{YMGRE_KEY_SPACE, 0, PAUSE, 0},
	{'n', 0, STEP, 0},
	{'r', 0, RESET, 0},
	{YMGRE_KEY_HOME, 0, RESET, 0},
	{'t', 0, AUTO, 0},
	{'c', 0, VIEW, 0},
	{'x', 0, SWITCH, 0},
	{'g', 0, POWER, 0},
	{'v', 0, WIND, 0},
	{YMGRE_KEY_ESCAPE, 0, QUIT, 0},
};

static GYOBJ label(GYOBJ root, int x, int y, int width, const char *text)
{
	GYOBJ o = YMGUI_Creat_Label_Creat(root, x, y, width, 24);
	if (o)
	{
		YMGUI_Label_SetText(o, text);
		YMGUI_Label_SetTextColor(o, GY_ARGB(255, 225, 235, 243));
	}
	return o;
}
static float axis(int held, int positive, int negative)
{
	return !!(held & positive) - !!(held & negative);
}
static int advance(simulation *s, int held)
{
	pme_aircraft *a = &s->owl.aircraft;
	pme_aircraft_command c = {axis(held, FORWARD, BACK), axis(held, RIGHT, LEFT),
							  axis(held, UP, DOWN), axis(held, YAWRIGHT, YAWLEFT), (float)s->power};
	if (s->automatic)
	{
		c.forward = 0;
		c.right = 0;
		c.turn = .65f;
		c.climb = YMPME_Clamp((18 - a->position.y) * .12f - a->velocity.y * .14f, -.5f, .5f);
	}
	pme_vec3 wind = s->wind ? YMPME_Vec3(2 + 1.2f * sinf(a->time * .7f), 0, .6f * cosf(a->time))
							: YMPME_Vec3(0, 0, 0);
	if (!OwlFlight_Step(&s->owl, &c, wind))
		return 0;
	s->command = c;
	s->steps++;
	return 1;
}
int main(int argc, char **argv)
{
	static simulation s;
	YMGRE_DemoHost host;
	int smoke = 0, ready = 0, ok = 0, frame = 0, checks = 0;
	const char *assets = NULL;
	for (int i = 1; i < argc; i++)
	{
		if (!strcmp(argv[i], "--smoke-test"))
			smoke = 1;
		else if (!strcmp(argv[i], "--assets") && i + 1 < argc)
			assets = argv[++i];
		else
		{
			fprintf(stderr, "Usage: %s [--assets directory] [--smoke-test]\n", argv[0]);
			return 2;
		}
	}
	if (!OwlFlight_Init(&s.owl, assets))
		goto done;
	s.power = 1;
	YMGRE_DemoHost_DefaultScale(1);
	if (!YMGRE_DemoHost_Init(&host, 1024, 800, 40))
		goto done;
	ready = 1;
	GYOBJ image = YMGRE_DemoHost_AddTarget(&host, YMGRE_Camera_GetRenderTarget(s.owl.visual.camera),
										   32, 40, 960, 640);
	GYOBJ status = label(host.context->root, 16, 6, 992, ""),
		  stats = label(host.context->root, 16, 685, 480, ""),
		  loads = label(host.context->root, 528, 685, 480, "");
	if (!image || !status || !stats || !loads ||
		!label(host.context->root, 16, 716, 480, "W/S Thrust | A/D Bank | UP/DOWN Pitch") ||
		!label(host.context->root, 528, 716, 480, "Q/E Turn | G Flap/Glide | T Auto/Manual") ||
		!label(host.context->root, 16, 745, 480, "C Camera | V Wind | X Wire | B Bones") ||
		!label(host.context->root, 528, 745, 480, "SPACE Pause | N Step | ESC Quit") ||
		!label(host.context->root, 16, 774, 480, "R/HOME Restart in air") ||
		!label(host.context->root, 528, 774, 480, "Hold keys to steer; release to level"))
		goto done;
	if (!YMGRE_DemoHost_BindKeys(&host, keyBindings, sizeof keyBindings / sizeof keyBindings[0]))
		goto done;

	unsigned savedSteps = 0;
	float savedHeading = 0;
	double previous = YMGRE_DemoHost_Time();
	puts(
		"Owl flight: keyboard manual control active. PME aircraft + imported YMSKE owl animation.");
	for (;; frame++)
	{
		if (smoke)
		{
			if (frame == 2)
			{
				if (!YMGRE_DemoHost_InjectKey(&host, 'w', 1) ||
					!YMGRE_DemoHost_InjectKey(&host, 'd', 1) ||
					!YMGRE_DemoHost_InjectKey(&host, YMGRE_KEY_UP, 1))
					goto done;
				savedHeading = s.owl.aircraft.q[1];
			}
			if (frame == 22)
			{
				if (s.automatic || s.command.forward != 1 || s.command.right != 1 ||
					s.command.climb != 1 || fabsf(s.owl.aircraft.q[1] - savedHeading) < .001f)
					goto done;
				checks |= 1;
				if (!YMGRE_DemoHost_InjectKey(&host, 'w', 0) ||
					!YMGRE_DemoHost_InjectKey(&host, 'd', 0) ||
					!YMGRE_DemoHost_InjectKey(&host, YMGRE_KEY_UP, 0) ||
					!YMGRE_DemoHost_InjectKey(&host, YMGRE_KEY_SPACE, 1))
					goto done;
				savedSteps = s.steps;
			}
			if (frame == 27)
			{
				if (!s.paused || s.steps != savedSteps || YMGRE_DemoHost_HeldKeys(&host))
					goto done;
				checks |= 2;
				if (!YMGRE_DemoHost_InjectKey(&host, 'n', 1))
					goto done;
			}
			if (frame == 29)
			{
				if (s.steps != savedSteps + 1)
					goto done;
				checks |= 4;
				if (!YMGRE_DemoHost_InjectKey(&host, YMGRE_KEY_SPACE, 1) ||
					!YMGRE_DemoHost_InjectKey(&host, 'g', 1))
					goto done;
			}
			if (frame == 90)
			{
				if (s.power || s.owl.aircraft.thrust != 0 || s.owl.flapBlend > .01f)
					goto done;
				checks |= 8;
				if (!YMGRE_DemoHost_InjectKey(&host, 't', 1) ||
					!YMGRE_DemoHost_InjectKey(&host, 'c', 1) ||
					!YMGRE_DemoHost_InjectKey(&host, 'v', 1) ||
					!YMGRE_DemoHost_InjectKey(&host, 'b', 1) ||
					!YMGRE_DemoHost_InjectKey(&host, 'x', 1))
					goto done;
			}
			if (frame == 94)
			{
				if (!s.automatic || s.view != 1 || !s.wind || !s.bones || !s.wire)
					goto done;
				checks |= 16;
				if (!YMGRE_DemoHost_InjectKey(&host, 'g', 1) ||
					!YMGRE_DemoHost_InjectKey(&host, 'c', 1) ||
					!YMGRE_DemoHost_InjectKey(&host, 'x', 1) ||
					!YMGRE_DemoHost_InjectKey(&host, 'b', 1))
					goto done;
			}
			if (frame == 114)
			{
				if (!s.power || s.owl.flapBlend < .8f || s.view != 2 || s.wire || s.bones)
					goto done;
				checks |= 32;
				if (!YMGRE_DemoHost_InjectKey(&host, 'w', 1))
					goto done;
				if (!YMGRE_DemoHost_InjectFocusLost(&host) || YMGRE_DemoHost_HeldKeys(&host))
					goto done;
				checks |= 64;
				if (!YMGRE_DemoHost_InjectKey(&host, YMGRE_KEY_HOME, 1))
					goto done;
			}
			if (frame == 118)
			{
				if (!s.power || s.wind || s.steps > 20)
					goto done;
				checks |= 128;
				if (!YMGRE_DemoHost_InjectKey(&host, YMGRE_KEY_ESCAPE, 1))
					goto done;
			}
		}
		int action = YMGRE_DemoHost_TakeActions(&host), held = YMGRE_DemoHost_HeldKeys(&host);
		if (action & QUIT)
		{
			ok = !smoke || checks == 255;
			break;
		}
		if (action & PAUSE)
		{
			s.paused = !s.paused;
			s.accumulator = 0;
		}
		if (action & AUTO)
			s.automatic = !s.automatic;
		if (action & POWER)
			s.power = !s.power;
		if (action & WIND)
			s.wind = !s.wind;
		if (action & VIEW)
			s.view = (s.view + 1) % 3;
		if (action & SWITCH)
			s.wire = !s.wire;
		if (action & CONTROL)
			s.bones = !s.bones;
		if (action & RESET)
		{
			if (!OwlFlight_Reset(&s.owl))
				goto done;
			s.steps = 0;
			s.accumulator = 0;
			s.power = 1;
			s.wind = 0;
			YMGRE_DemoHost_ClearInput(&host);
			held = 0;
		}
		double now = YMGRE_DemoHost_Time();
		double dt = smoke ? 1. / 60 : (double)(now - previous);
		previous = now;
		if ((action & STEP) && s.paused && !advance(&s, held))
			goto done;
		if (!s.paused)
		{
			s.accumulator += fmin(dt, .1);
			while (s.accumulator >= 1. / 240)
			{
				if (!advance(&s, held))
					goto done;
				s.accumulator -= 1. / 240;
			}
		}
		if (!OwlFlight_Render(&s.owl, s.view, s.wire, s.bones))
			goto done;
		YMGUI_Obj_Invalidate(image);
		if (frame % 4 == 0 || action)
		{
			pme_aircraft *a = &s.owl.aircraft;
			char text[256];
			snprintf(text, sizeof text, "OWL | %s | %s | %s | %s | %s",
					 s.automatic ? "AUTO" : "KEYBOARD", s.paused ? "PAUSED" : "RUNNING",
					 s.power ? "FLAPPING" : "GLIDING", s.wind ? "GUSTS" : "CALM",
					 a->grounded ? "LANDED: R" : "AIRBORNE");
			YMGUI_Label_SetText(status, text);
			snprintf(text, sizeof text, "Height %.1f m | Air %.1f m/s | VY %+.1f", a->position.y,
					 a->airspeed, a->velocity.y);
			YMGUI_Label_SetText(stats, text);
			snprintf(text, sizeof text, "Lift %.1f N | Thrust %.1f N | %.1f s", a->lift, a->thrust,
					 a->time);
			YMGUI_Label_SetText(loads, text);
		}

		if (frame < 3)
			YMGRE_DemoHost_SetTitle(&host, "YMPME | Owl Flight - Keyboard Control");
		if (!YMGRE_DemoHost_Step(&host, smoke ? 0 : 60))
		{
			ok = !smoke;
			break;
		}
	}
done:

	if (ready)
		YMGRE_DemoHost_Destroy(&host);
	OwlFlight_Destroy(&s.owl);
	if (smoke)
		printf("OWL_FLIGHT_WINDOW %s frame=%d checks=%d\n", ok ? "PASS" : "FAIL", frame, checks);
	return !ok;
}
