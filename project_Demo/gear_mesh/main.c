#include "scene.h"
#include "demo_host.h"
#include "YMGUI_Label.h"
#include "YMGUI_Invalidate.h"
#include <stdio.h>
#include <string.h>
enum
{
	PAUSE = 1,
	REVERSE = 2,
	UP = 4,
	DOWN = 8,
	CAMERA = 16,
	GUIDE = 32,
	WIRE = 64,
	MOTOR = 128,
	RESET = 256,
	STEP = 512,
	QUIT = 1024,
	AUTO = 2048
};
static const YMGRE_DemoKeyBinding keyBindings[] = {
	{'t', 0, AUTO, 0},
	{YMGRE_KEY_SPACE, 0, PAUSE, 0},
	{'r', 0, REVERSE, 0},
	{YMGRE_KEY_UP, 0, UP, 0},
	{YMGRE_KEY_DOWN, 0, DOWN, 0},
	{'c', 0, CAMERA, 0},
	{'g', 0, GUIDE, 0},
	{'v', 0, WIRE, 0},
	{'m', 0, MOTOR, 0},
	{YMGRE_KEY_HOME, 0, RESET, 0},
	{'n', 0, STEP, 0},
	{YMGRE_KEY_ESCAPE, 0, QUIT, 0},
};

static GYOBJ label(GYOBJ root, int x, int y, int width, const char *text)
{
	GYOBJ p = YMGUI_Creat_Label_Creat(root, x, y, width, 22);
	if (p)
	{
		YMGUI_Label_SetText(p, text);
		YMGUI_Label_SetTextColor(p, GY_ARGB(255, 227, 233, 239));
	}
	return p;
}
static uint64_t hash(GRE_RenderTarget t)
{
	size_t i, n = (size_t)t->width * t->height * sizeof(*t->data);
	const unsigned char *b = (const unsigned char *)t->data;
	uint64_t h = UINT64_C(14695981039346656037);
	for (i = 0; i < n; ++i)
		h = (h ^ b[i]) * UINT64_C(1099511628211);
	return h;
}
static int advance(gear_scene *s, double dt)
{
	int i;
	for (i = 0; i < 3; ++i)
		if (!YMPME_Gear_Step(&s[i].pair, dt))
			return 0;
	return 1;
}
int main(int argc, char **argv)
{
	static gear_scene s[3];
	YMGRE_DemoHost host;

	GYOBJ image[3], rpm[3], contact[3], phase[3], info;
	const char *names[] = {"1  IDEAL CONSTRAINT", "2  TOOTH COLLISION / TIGHT",
						   "3  TOOTH COLLISION / BACKLASH"};
	int smoke = argc == 2 && !strcmp(argv[1], "--smoke-test"), ready = 0, result = 1;
	int frame = 0, paused = 0, view = 0, guides = 1, wire = 0, changed = 0, title = 0, single = 0,
		reversed = 0, automatic = 0, autoSeen = 0;
	int i, collided[3] = {0};
	double previous;
	double accumulator = 0, saved[3], autoClock = 0;
	uint64_t firstHash[3] = {0};
	if (argc > 1 && !smoke)
	{
		fprintf(stderr, "Usage: %s [--smoke-test]\n", argv[0]);
		return 1;
	}
	for (i = 0; i < 3; ++i)
		if (!GearScene_InitMode(&s[i], (pme_gear_mode)i, 420, 360))
			goto done;
	YMGRE_DemoHost_DefaultScale(1);
	if (!YMGRE_DemoHost_Init(&host, 1260, 680, 40))
	{
		fprintf(stderr, "GRE host: %s\n", YMGRE_DemoHost_LastError());
		goto done;
	}
	ready = 1;
	for (i = 0; i < 3; ++i)
	{
		if (!label(host.context->root, i * 420, 4, 420, names[i]))
			goto done;
		image[i] = YMGRE_DemoHost_AddTarget(&host, YMGRE_Camera_GetRenderTarget(s[i].camera),
											i * 420, 30, 420, 360);
		rpm[i] = label(host.context->root, i * 420, 396, 420, "Input / output RPM");
		contact[i] = label(host.context->root, i * 420, 420, 420,
						   i == 0 ? "Exact angular ratio" : "AABB -> tooth SAT -> angular impulse");
		phase[i] = label(host.context->root, i * 420, 444, 420, "Transmission phase error");
		if (!image[i] || !rpm[i] || !contact[i] || !phase[i])
			goto done;
	}
	info = label(host.context->root, 0, 490, 1260, "Synchronized controls for all three pairs");
	if (!info ||
		!label(host.context->root, 0, 516, 1260,
			   "LEFT Ideal constraint    CENTER 3.00m centers    RIGHT 3.08m") ||
		!label(host.context->root, 0, 544, 1260,
			   "UP/DOWN Speed   R Reverse   SPACE Pause   N Single step") ||
		!label(host.context->root, 0, 572, 1260,
			   "T Auto reverse test (4 RPM)   C Angled/Top/Close") ||
		!label(host.context->root, 0, 600, 1260, "G Pitch circles + red contacts   V Wireframe") ||
		!label(host.context->root, 0, 628, 1260, "M Motor/coast   HOME Reset   ESC Exit"))
		goto done;
	if (!YMGRE_DemoHost_BindKeys(&host, keyBindings, sizeof keyBindings / sizeof keyBindings[0]))
		goto done;

	previous = YMGRE_DemoHost_Time();

	puts("Three gears: ideal / tight tooth collision / backlash. T auto reverse; R reverse; C "
		 "views; SPACE "
		 "pause; N step.");
	fflush(stdout);
	for (;; ++frame)
	{
		double now = YMGRE_DemoHost_Time();
		double elapsed = smoke ? 1.0 / 60 : (double)(now - previous);
		int a, steps = 0;
		previous = now;
		if (!title)
		{

			if (host.context)
			{
				YMGRE_DemoHost_SetTitle(
					&host, "YMPME + GRE | Constraint vs tooth collisions | Tight vs backlash");
				title = 1;
			}
		}
		if (smoke)
		{
			if (frame == 30)
			{
				for (i = 0; i < 3; ++i)
					saved[i] = s[i].pair.angle;
				if (!YMGRE_DemoHost_InjectKey(&host, YMGRE_KEY_SPACE, 1))
					goto done;
			}
			if (frame == 40)
			{
				for (i = 0; i < 3; ++i)
					if (s[i].pair.angle != saved[i])
						goto done;
				if (!YMGRE_DemoHost_InjectKey(&host, 'n', 1))
					goto done;
			}
			if (frame == 41)
			{
				for (i = 0; i < 3; ++i)
					if (s[i].pair.angle == saved[i])
						goto done;
				single = 1;
			}
			if (frame == 45 && !YMGRE_DemoHost_InjectKey(&host, YMGRE_KEY_SPACE, 1))
				goto done;
			if (frame == 50 && (!YMGRE_DemoHost_InjectKey(&host, 'r', 1) ||
								!YMGRE_DemoHost_InjectKey(&host, YMGRE_KEY_UP, 1)))
				goto done;
			if (frame == 110)
			{
				for (i = 0; i < 3; ++i)
					if (s[i].pair.omega >= 0 || YMPME_Gear_DrivenOmega(&s[i].pair) <= 0)
						goto done;
				reversed = 1;
				if (!YMGRE_DemoHost_InjectKey(&host, 'm', 1))
					goto done;
			}
			if (frame == 120 && !YMGRE_DemoHost_InjectKey(&host, 'c', 1))
				goto done;
			if (frame == 130 && (!YMGRE_DemoHost_InjectKey(&host, 'c', 1) ||
								 !YMGRE_DemoHost_InjectKey(&host, 'g', 1)))
				goto done;
			if (frame == 140 && !YMGRE_DemoHost_InjectKey(&host, 'v', 1))
				goto done;
			if (frame == 145 && !YMGRE_DemoHost_InjectKey(&host, 't', 1))
				goto done;
			if (frame == 150 && !YMGRE_DemoHost_InjectKey(&host, YMGRE_KEY_HOME, 1))
				goto done;
			if (frame == 160 && !YMGRE_DemoHost_InjectKey(&host, YMGRE_KEY_ESCAPE, 1))
				goto done;
		}
		a = YMGRE_DemoHost_TakeActions(&host);
		if (a & QUIT)
		{
			result = smoke ? !(single && reversed && changed == 7 && view == 2 && !guides && wire &&
							   autoSeen && collided[1] && collided[2])
						   : 0;
			break;
		}
		if (a & RESET)
		{
			for (i = 0; i < 3; ++i)
				YMPME_Gear_ResetMode(&s[i].pair, (pme_gear_mode)i);
			accumulator = 0;
			elapsed = 0;
			paused = 0;
			automatic = 0;
		}
		if (a & AUTO)
		{
			automatic = !automatic;
			autoClock = 0;
			autoSeen = 1;
			if (automatic)
			{
				for (i = 0; i < 3; ++i)
				{
					YMPME_Gear_ResetMode(&s[i].pair, (pme_gear_mode)i);
					s[i].pair.target = 4 * 2 * PME_GEAR_PI / 60;
				}
				paused = 0;
				accumulator = 0;
				elapsed = 0;
			}
		}
		if (a & PAUSE)
		{
			paused = !paused;
			elapsed = 0;
		}
		for (i = 0; i < 3; ++i)
		{
			pme_gear_pair *p = &s[i].pair;
			if (a & REVERSE)
			{
				p->target = -p->target;
				automatic = 0;
			}
			if (a & (UP | DOWN))
			{
				double speed = fabs(p->target) * 60 / (2 * PME_GEAR_PI);
				speed = fmax(0, fmin(60, speed + ((a & UP) ? 2 : -2)));
				p->target = copysign(speed * 2 * PME_GEAR_PI / 60, p->target);
				automatic = 0;
			}
			if (a & MOTOR)
			{
				p->motor = !p->motor;
				automatic = 0;
			}
		}
		if (a & CAMERA)
			view = (view + 1) % 3;
		if (a & GUIDE)
			guides = !guides;
		if (a & WIRE)
			wire = !wire;
		if (paused && (a & STEP) && !advance(s, 1.0 / 120))
			goto done;
		if (!paused)
		{
			accumulator += fmin(elapsed, .25);
			while (accumulator >= 1.0 / 120 && steps++ < 24)
			{
				if (automatic)
				{
					autoClock += 1.0 / 120;
					if (autoClock >= 3)
					{
						for (i = 0; i < 3; ++i)
							s[i].pair.target = -s[i].pair.target;
						autoClock = 0;
					}
				}
				if (!advance(s, 1.0 / 120))
					goto done;
				accumulator -= 1.0 / 120;
				for (i = 1; i < 3; ++i)
					if (s[i].pair.contacts)
						collided[i] = 1;
			}
			if (accumulator >= 1.0 / 120)
				accumulator = fmod(accumulator, 1.0 / 120);
		}
		for (i = 0; i < 3; ++i)
		{
			GearScene_Render(&s[i], view, guides, wire);
			YMGUI_Obj_Invalidate(image[i]);
			if (smoke)
			{
				uint64_t h = hash(YMGRE_Camera_GetRenderTarget(s[i].camera));
				if (!frame)
					firstHash[i] = h;
				else if (h != firstHash[i])
					changed |= 1 << i;
			}
		}
		if (frame % 4 == 0 || a)
		{
			char text[64];
			for (i = 0; i < 3; ++i)
			{
				pme_gear_pair *p = &s[i].pair;
				double error = remainder(YMPME_Gear_DrivenAngle(p) + p->angle * 2 / 3 -
											 (PME_GEAR_PI - PME_GEAR_PI / 30),
										 2 * PME_GEAR_PI / 30) *
							   180 / PME_GEAR_PI;
				snprintf(text, sizeof(text), "RPM  A %+.2f    B %+.2f",
						 p->omega * 60 / (2 * PME_GEAR_PI),
						 YMPME_Gear_DrivenOmega(p) * 60 / (2 * PME_GEAR_PI));
				YMGUI_Label_SetText(rpm[i], text);
				if (i)
					snprintf(text, sizeof(text), "Teeth %d | Contact torque %+.2f Nm", p->contacts,
							 p->contactTorque);
				else
					snprintf(text, sizeof(text), "Ratio -2/3 | Fixed angular constraint");
				YMGUI_Label_SetText(contact[i], text);
				snprintf(text, sizeof(text), "Phase error %+.3f deg", error);
				YMGUI_Label_SetText(phase[i], text);
			}
			snprintf(text, sizeof(text), "%s | %s | Target %+.0f RPM | Same commands / same load",
					 paused		 ? "PAUSED"
					 : automatic ? "AUTO"
								 : "RUN",
					 s[0].pair.motor ? "MOTOR" : "COAST",
					 s[0].pair.target * 60 / (2 * PME_GEAR_PI));
			YMGUI_Label_SetText(info, text);
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
	for (i = 0; i < 3; ++i)
		GearScene_Destroy(&s[i]);
	if (smoke)
		printf(
			"%s three gears: tooth contacts, reverse, pause, single step, render, comparison UI\n",
			result ? "FAIL" : "PASS");
	return result;
}
