#include "YMGRE_Camera.h"
#include "YMGUI_Invalidate.h"
#include "YMGUI_Label.h"
#include "demo_host.h"
#include "scene.h"
#ifdef PME_DEMO_CONTROL
#include "control_rig.h"
#define CONTROL_HINT "B Drone: external PID / built-in  HOME Reset  ESC Exit"
#else
#define CONTROL_HINT "HOME Reset  ESC Exit"
#endif
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
	flight_scene scene;
	int paused, automatic, selected, view, wind;
	int power[2];
	double accumulator;
	unsigned steps;
#ifdef PME_DEMO_CONTROL
	drone_control_rig rig;
	int external;
#endif
} simulation;
static const YMGRE_DemoKeyBinding keyBindings[] = {
	{'w', FORWARD, 0, 0},
	{'s', BACK, 0, 0},
	{'a', LEFT, 0, 0},
	{'d', RIGHT, 0, 0},
	{'r', UP, 0, 0},
	{'f', DOWN, 0, 0},
	{'q', YAWLEFT, 0, 0},
	{'e', YAWRIGHT, 0, 0},
	{'b', 0, CONTROL, 0},
	{YMGRE_KEY_SPACE, 0, PAUSE, 0},
	{'n', 0, STEP, 0},
	{YMGRE_KEY_HOME, 0, RESET, 0},
	{'t', 0, AUTO, 0},
	{'c', 0, VIEW, 0},
	{YMGRE_KEY_TAB, 0, SWITCH, 0},
	{'g', 0, POWER, 0},
	{'v', 0, WIND, 0},
	{YMGRE_KEY_ESCAPE, 0, QUIT, 0},
};

static GYOBJ label(GYOBJ root, int x, int y, int width, const char *text, GYcolor color)
{
	GYOBJ p = YMGUI_Creat_Label_Creat(root, x, y, width, 23);
	if (p)
	{
		YMGUI_Label_SetText(p, text);
		YMGUI_Label_SetTextColor(p, color);
	}
	return p;
}
static float axis(int held, int pos, int neg)
{
	return (held & pos ? 1.0f : 0) - (held & neg ? 1.0f : 0);
}
static int advance(simulation *s, int held)
{
	for (int i = 0; i < 2; ++i)
	{
		pme_aircraft *a = &s->scene.aircraft[i];
		pme_aircraft_command c = {0, 0, 0, 0, (float)s->power[i]};
		if (s->automatic || s->selected != i)
		{
			float t = (float)a->time;
			if (i == 0)
			{
				pme_vec3 f = YMPME_Aircraft_Rotate(a, YMPME_Vec3(0, 0, 1));
				float yaw = atan2f(f.x, f.z);
				float vx = .9f * cosf(t * .3f) + .8f * (-6 + 3 * sinf(t * .3f) - a->position.x);
				float vz = .9f * sinf(t * .3f) + .8f * (3 - 3 * cosf(t * .3f) - a->position.z);
				c.forward = YMPME_Clamp((vx * sinf(yaw) + vz * cosf(yaw)) / 5, -1, 1);
				c.right = YMPME_Clamp((vx * cosf(yaw) - vz * sinf(yaw)) / 5, -1, 1);
				c.climb = YMPME_Clamp((5 + .8f * sinf(t * .6f) - a->position.y) * .6f, -.8f, .8f);
				c.turn = .18f;
			}
			else
			{
				c.turn = .85f;
				c.climb =
					YMPME_Clamp((12 - a->position.y) * .12f - a->velocity.y * .14f, -.5f, .5f);
			}
		}
		else
		{
			c.forward = axis(held, FORWARD, BACK);
			c.right = axis(held, RIGHT, LEFT);
			c.climb = axis(held, UP, DOWN);
			c.turn = axis(held, YAWRIGHT, YAWLEFT);
		}
		pme_vec3 wind = s->wind ? YMPME_Vec3(2 + 1.2f * sinf((float)a->time * .7f), 0,
											 .6f * cosf((float)a->time))
								: YMPME_Vec3(0, 0, 0);
#ifdef PME_DEMO_CONTROL
		if (i == 0 && s->external)
		{
			if (!DroneControlRig_Step(&s->rig, a, &c, wind, 1.0f / 240))
				return 0;
			continue;
		}
#endif
		if (YMPME_Aircraft_Step(a, &c, wind, 1.0f / 240) != PME_OK)
			return 0;
	}
	++s->steps;
	return 1;
}
static uint64_t hash(GRE_RenderTarget t)
{
	uint64_t h = UINT64_C(14695981039346656037);
	const unsigned char *p = (const unsigned char *)t->data;
	size_t n = (size_t)t->width * t->height * sizeof(*t->data);
	for (size_t i = 0; i < n; i += 13)
		h = (h ^ p[i]) * UINT64_C(1099511628211);
	return h;
}
int main(int argc, char **argv)
{
	static simulation s;
	YMGRE_DemoHost host;

	GYOBJ image[2], stats[2][3], status;
	int smoke = argc == 2 && !strcmp(argv[1], "--smoke-test"), ready = 0, result = 1, frame = 0,
		title = 0, changed = 0, tested = 0;
	unsigned savedSteps = 0;
	float savedHeight = 0;
	uint64_t first[2] = {0};
	if (argc > 1 && !smoke)
	{
		fprintf(stderr, "Usage: %s [--smoke-test]\n", argv[0]);
		return 1;
	}
	s.automatic = 1;
	s.power[0] = s.power[1] = 1;
#ifdef PME_DEMO_CONTROL
	DroneControlRig_Init(&s.rig);
	s.external = 1;
#endif
	if (!FlightScene_Init(&s.scene))
		goto done;
	YMGRE_DemoHost_DefaultScale(1);
	if (!YMGRE_DemoHost_Init(&host, 1200, 700, 40))
		goto done;
	ready = 1;
	const char *names[2] = {"01  QUADCOPTER / STABILIZED FLIGHT", "02  EAGLE / FLAP & GLIDE"};
	for (int i = 0; i < 2; ++i)
	{
		if (!label(host.context->root, 12 + i * 600, 9, 585, names[i],
				   i ? GY_ARGB(255, 244, 198, 104) : GY_ARGB(255, 94, 218, 235)))
			goto done;
		image[i] = YMGRE_DemoHost_AddTarget(&host, YMGRE_Camera_GetRenderTarget(s.scene.camera[i]),
											i * 600, 40, 600, 450);
		if (!image[i])
			goto done;
		for (int j = 0; j < 3; ++j)
		{
			stats[i][j] = label(host.context->root, 12 + i * 600, 502 + j * 25, 585, "",
								GY_ARGB(255, 222, 233, 241));
			if (!stats[i][j])
				goto done;
		}
	}
	status = label(host.context->root, 12, 586, 1180, "", GY_ARGB(255, 139, 218, 173));
	if (!status ||
		!label(host.context->root, 12, 619, 580, "TAB Select  T Auto/manual  WASD Move/bank",
			   GY_ARGB(255, 218, 226, 237)) ||
		!label(host.context->root, 612, 619, 580, "R/F Rise/dive  Q/E Turn  G Motors/flapping",
			   GY_ARGB(255, 218, 226, 237)) ||
		!label(host.context->root, 12, 650, 580, "V Wind  C Camera  SPACE Pause  N Step",
			   GY_ARGB(255, 218, 226, 237)) ||
		!label(host.context->root, 612, 650, 580, CONTROL_HINT, GY_ARGB(255, 218, 226, 237)))
		goto done;
	if (!YMGRE_DemoHost_BindKeys(&host, keyBindings, sizeof keyBindings / sizeof keyBindings[0]))
		goto done;

	double previous = YMGRE_DemoHost_Time();
	puts("Flight playground: TAB select; T manual; G selected drone motors / "
		 "eagle flapping. Ground support only; no aircraft-to-aircraft "
		 "collision.");
	fflush(stdout);
	for (;; ++frame)
	{
		double now = YMGRE_DemoHost_Time();
		double elapsed = smoke ? 1.0 / 60 : (double)(now - previous);
		previous = now;
		if (!title)
		{

			if (host.context)
			{
				YMGRE_DemoHost_SetTitle(&host,
										"YMPME + GRE | Quadcopter & Eagle Flight Playground");
				title = 1;
			}
		}
		if (smoke)
		{
#ifdef PME_DEMO_CONTROL
			if (frame == 4 && !YMGRE_DemoHost_InjectKey(&host, 'b', 1))
				goto done;
			if (frame == 6 && s.external)
				goto done;
			if (frame == 8 && !YMGRE_DemoHost_InjectKey(&host, 'b', 1))
				goto done;
			if (frame == 10 && (!s.external || !s.rig.updates))
				goto done;
#endif
			if (frame == 12)
			{
				savedSteps = s.steps;
				if (!YMGRE_DemoHost_InjectKey(&host, YMGRE_KEY_SPACE, 1))
					goto done;
			}
			if (frame == 18)
			{
				if (!s.paused || s.steps != savedSteps)
					goto done;
				tested |= 1;
				if (!YMGRE_DemoHost_InjectKey(&host, 'n', 1))
					goto done;
			}
			if (frame == 20)
			{
				if (s.steps != savedSteps + 1)
					goto done;
				tested |= 2;
				if (!YMGRE_DemoHost_InjectKey(&host, YMGRE_KEY_SPACE, 1) ||
					!YMGRE_DemoHost_InjectKey(&host, 't', 1) ||
					!YMGRE_DemoHost_InjectKey(&host, 'r', 1))
					goto done;
				savedHeight = s.scene.aircraft[0].position.y;
			}
			if (frame == 70)
			{
				if (s.automatic || s.scene.aircraft[0].position.y < savedHeight + .1f)
					goto done;
				tested |= 4;
				if (!YMGRE_DemoHost_InjectKey(&host, 'r', 0) ||
					!YMGRE_DemoHost_InjectKey(&host, YMGRE_KEY_TAB, 1) ||
					!YMGRE_DemoHost_InjectKey(&host, 'g', 1))
					goto done;
			}
			if (frame == 100)
			{
				if (s.selected != 1 || s.power[1] || s.scene.aircraft[1].thrust != 0)
					goto done;
				tested |= 8;
				if (!YMGRE_DemoHost_InjectKey(&host, 'v', 1) ||
					!YMGRE_DemoHost_InjectKey(&host, 'c', 1) ||
					!YMGRE_DemoHost_InjectKey(&host, 't', 1))
					goto done;
			}
			if (frame == 108)
			{
				if (!s.wind || s.view != 1 || !s.automatic)
					goto done;
				tested |= 16;
				if (!YMGRE_DemoHost_InjectKey(&host, YMGRE_KEY_HOME, 1))
					goto done;
			}
			if (frame == 110)
			{
				if (s.steps > 8 || !s.power[1] || s.wind)
					goto done;
				tested |= 32;
				if (!YMGRE_DemoHost_InjectKey(&host, YMGRE_KEY_ESCAPE, 1))
					goto done;
			}
		}
		int action = YMGRE_DemoHost_TakeActions(&host), held = YMGRE_DemoHost_HeldKeys(&host);
		if (action & QUIT)
		{
			result = smoke ? !(tested == 63 && changed == 3) : 0;
			break;
		}
#ifdef PME_DEMO_CONTROL
		if (action & CONTROL)
		{
			s.external = !s.external;
			DroneControlRig_Init(&s.rig);
		}
#endif
		if (action & PAUSE)
		{
			s.paused = !s.paused;
			s.accumulator = 0;
		}
		if (action & SWITCH)
			s.selected = 1 - s.selected;
		if (action & AUTO)
			s.automatic = !s.automatic;
		if (action & VIEW)
			s.view = (s.view + 1) % 3;
		if (action & POWER)
			s.power[s.selected] = !s.power[s.selected];
		if (action & WIND)
			s.wind = !s.wind;
		if (action & RESET)
		{
#ifdef PME_DEMO_CONTROL
			DroneControlRig_Init(&s.rig);
#endif
			if (!FlightScene_Reset(&s.scene))
				goto done;
			s.accumulator = 0;
			s.steps = 0;
			s.power[0] = s.power[1] = 1;
			s.wind = 0;
			YMGRE_DemoHost_ClearInput(&host);
			held = 0;
		}
		if ((action & STEP) && s.paused)
		{
			if (!advance(&s, held))
				goto done;
		}
		if (!s.paused)
		{
			s.accumulator += fmin(elapsed, .1);
			int n = 0;
			while (s.accumulator >= 1.0 / 240 && n < 24)
			{
				if (!advance(&s, held))
					goto done;
				s.accumulator -= 1.0 / 240;
				++n;
			}
		}
		FlightScene_Render(&s.scene, s.view);
		for (int i = 0; i < 2; ++i)
		{
			YMGUI_Obj_Invalidate(image[i]);
			if (smoke)
			{
				uint64_t h = hash(YMGRE_Camera_GetRenderTarget(s.scene.camera[i]));
				if (!frame)
					first[i] = h;
				else if (h != first[i])
					changed |= 1 << i;
			}
		}
		if (frame % 4 == 0 || action)
		{
			char text[150];
			for (int i = 0; i < 2; ++i)
			{
				pme_aircraft *a = &s.scene.aircraft[i];
				snprintf(text, sizeof(text), "%s | %s | Altitude %.2f m | Airspeed %.2f m/s",
						 s.selected == i ? "SELECTED" : "AUTO",
						 i ? (s.power[i] ? "FLAPPING" : "GLIDING")
						   : (s.power[i] ? "MOTORS ON" : "MOTORS OFF"),
						 a->position.y, a->airspeed);
				YMGUI_Label_SetText(stats[i][0], text);
				snprintf(text, sizeof(text), "Lift %.1f N | Thrust %.1f N | Vertical %+.2f m/s",
						 a->lift, a->thrust, a->velocity.y);
				YMGUI_Label_SetText(stats[i][1], text);
				if (i)
					snprintf(text, sizeof(text), "Wing %.0f deg | %s | Mass %.1f kg",
							 a->wingAngle * 57.2958f, a->grounded ? "GROUNDED" : "AIRBORNE",
							 a->mass);
				else
					snprintf(text, sizeof(text), "Rotor RPM %.0f / %.0f / %.0f / %.0f",
							 a->rotorSpeed[0] * 9.5493f, a->rotorSpeed[1] * 9.5493f,
							 a->rotorSpeed[2] * 9.5493f, a->rotorSpeed[3] * 9.5493f);
				YMGUI_Label_SetText(stats[i][2], text);
#ifdef PME_DEMO_CONTROL
				if (i == 0 && s.external)
				{
					snprintf(text, sizeof(text), "IMU ay %.2f | gyro Y %+.3f | F Y %.1f N",
							 s.rig.imu.specificForce.y, s.rig.imu.angularVelocity.y,
							 s.rig.force.wrench.force.y);
					YMGUI_Label_SetText(stats[i][1], text);
					snprintf(text, sizeof(text), "EXT PID | ENC RPM %.0f %.0f %.0f %.0f",
							 fabsf(s.rig.enc[0].velocity) * 9.5493f,
							 fabsf(s.rig.enc[1].velocity) * 9.5493f,
							 fabsf(s.rig.enc[2].velocity) * 9.5493f,
							 fabsf(s.rig.enc[3].velocity) * 9.5493f);
					YMGUI_Label_SetText(stats[i][2], text);
				}
#endif
			}
			snprintf(text, sizeof(text), "%s %s | %s | Wind %s | C%d | %.1fs",
					 s.paused ? "PAUSED" : "RUNNING", s.automatic ? "AUTO TOUR" : "MANUAL",
					 s.selected ? "EAGLE" : "DRONE", s.wind ? "GUSTS" : "CALM", s.view + 1,
					 s.scene.aircraft[0].time);
			YMGUI_Label_SetText(status, text);
		}
		if (!YMGRE_DemoHost_Step(&host, smoke ? 0 : 60))
		{
			result = smoke ? 1 : 0;
			break;
		}
	}
done:

	if (ready)
		YMGRE_DemoHost_Destroy(&host);
	FlightScene_Destroy(&s.scene);
	if (smoke)
		printf("%s flight window: changed=%d controls=%d frame=%d\n", result ? "FAIL" : "PASS",
			   changed, tested, frame);
	return result;
}
