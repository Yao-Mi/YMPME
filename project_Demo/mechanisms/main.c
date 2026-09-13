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
	REVERSE = 4,
	MOTOR = 8,
	FRICTION = 16,
	HOLD = 32,
	KICK = 64,
	CAMERA = 128,
	CONTACT = 256,
	RESET = 512,
	QUIT = 1024,
	UP = 2048,
	DOWN = 4096,
	AUTO = 8192
};

static const YMGRE_DemoKeyBinding keyBindings[] = {
	{YMGRE_KEY_SPACE, 0, PAUSE, 0},
	{'n', 0, STEP, 0},
	{'r', 0, REVERSE, 0},
	{'m', 0, MOTOR, 0},
	{'f', 0, FRICTION, 0},
	{'h', 0, HOLD, 0},
	{'k', 0, KICK, 0},
	{'c', 0, CAMERA, 0},
	{'g', 0, CONTACT, 0},
	{YMGRE_KEY_HOME, 0, RESET, 0},
	{YMGRE_KEY_ESCAPE, 0, QUIT, 0},
	{YMGRE_KEY_UP, 0, UP, 0},
	{YMGRE_KEY_DOWN, 0, DOWN, 0},
	{'t', 0, AUTO, 0},
	{'w', 1, 0, 0},
	{'s', 2, 0, 0},
};

static GYOBJ label(GYOBJ root, int x, int y, int w, const char *t)
{
	GYOBJ p = YMGUI_Creat_Label_Creat(root, x, y, w, 22);
	if (p)
	{
		YMGUI_Label_SetText(p, t);
		YMGUI_Label_SetTextColor(p, GY_ARGB(255, 227, 233, 239));
	}
	return p;
}
static uint64_t hash(GRE_RenderTarget t)
{
	size_t n = (size_t)t->width * t->height * sizeof(*t->data);
	const unsigned char *p = (const unsigned char *)t->data;
	uint64_t h = UINT64_C(14695981039346656037);
	for (size_t i = 0; i < n; ++i)
		h = (h ^ p[i]) * UINT64_C(1099511628211);
	return h;
}
int main(int argc, char **argv)
{
	static mech_scene scenes[4];
	mechanisms_model m;
	YMGRE_DemoHost host;

	GYOBJ images[4], stats[4][3], info;
	const char *names[] = {"1  BELT / IDEAL NO-SLIP CONSTRAINT", "2  BELT / CONTACT + FRICTION",
						   "3  BLOCK AND TACKLE / 4:1 HOIST", "4  ROPE-END MASS / FREE SWING"};
	int smoke = argc == 2 && !strcmp(argv[1], "--smoke-test"), ready = 0, result = 1;
	int frame = 0, paused = 0, view = 0, contacts = 1, automatic = 1, title = 0, changed = 0,
		tested = 0;
	double accumulator = 0, saved = 0, autoClock = 0, savedHeight = 0;
	uint64_t first[4] = {0};
	double previous;
	if (argc > 1 && !smoke)
	{
		fprintf(stderr, "Usage: %s [--smoke-test]\n", argv[0]);
		return 1;
	}
	Mechanisms_Init(&m);
	for (int i = 0; i < 4; ++i)
		if (!MechScene_Init(&scenes[i], i, 600, 310))
			goto done;
	YMGRE_DemoHost_DefaultScale(1);
	if (!YMGRE_DemoHost_Init(&host, 1200, 900, 40))
	{
		fprintf(stderr, "GRE host: %s\n", YMGRE_DemoHost_LastError());
		goto done;
	}
	ready = 1;
	for (int i = 0; i < 4; ++i)
	{
		int x = (i % 2) * 600, y = (i / 2) * 408;
		if (!label(host.context->root, x, y + 4, 600, names[i]))
			goto done;
		images[i] = YMGRE_DemoHost_AddTarget(&host, YMGRE_Camera_GetRenderTarget(scenes[i].camera),
											 x, y + 30, 600, 310);
		if (!images[i])
			goto done;
		for (int j = 0; j < 3; ++j)
			if (!(stats[i][j] = label(host.context->root, x, y + 344 + j * 21, 600, "")))
				goto done;
	}
	info = label(host.context->root, 0, 818, 1200, "");
	if (!info ||
		!label(host.context->root, 0, 844, 600, "W/S Lift/lower  H Hold/release  K Push swing") ||
		!label(host.context->root, 600, 844, 600, "F Friction  T Auto  C View  G Contacts") ||
		!label(host.context->root, 0, 870, 600, "UP/DOWN Speed  R Reverse  M Motor/coast") ||
		!label(host.context->root, 600, 870, 600, "SPACE Pause  N Step  HOME Reset  ESC Exit"))
		goto done;
	if (!YMGRE_DemoHost_BindKeys(&host, keyBindings, sizeof keyBindings / sizeof keyBindings[0]))
		goto done;

	previous = YMGRE_DemoHost_Time();

	puts("YMPME mechanisms: ideal belt / contact belt / 4:1 hoist / rope-end swing. W/S lift; F "
		 "friction; T "
		 "auto; SPACE pause.");
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
				YMGRE_DemoHost_SetTitle(&host, "YMPME + GRE | Belts, pulleys and rope-end swing");
				title = 1;
			}
		}
		if (smoke)
		{
			if (frame == 20)
			{
				saved = m.time;
				if (!YMGRE_DemoHost_InjectKey(&host, YMGRE_KEY_SPACE, 1))
					goto done;
			}
			if (frame == 25)
			{
				if (m.time != saved)
					goto done;
				tested |= 1;
				if (!YMGRE_DemoHost_InjectKey(&host, 'n', 1))
					goto done;
			}
			if (frame == 26)
			{
				if (fabs(m.time - saved - 1.0 / 120) > 1e-9)
					goto done;
				tested |= 2;
			}
			if (frame == 30 && (!YMGRE_DemoHost_InjectKey(&host, YMGRE_KEY_SPACE, 1) ||
								!YMGRE_DemoHost_InjectKey(&host, 'r', 1)))
				goto done;
			if (frame == 35)
			{
				if (m.belt[1].target <= 0 || automatic)
					goto done;
				tested |= 4;
				if (!YMGRE_DemoHost_InjectKey(&host, 'f', 1))
					goto done;
			}
			if (frame == 36)
			{
				if (m.belt[1].mu != 0)
					goto done;
				tested |= 8;
			}
			if (frame == 40)
			{
				savedHeight = m.hoist.loadY;
				if (!YMGRE_DemoHost_InjectKey(&host, 'w', 1))
					goto done;
			}
			if (frame == 60)
			{
				if (m.hoist.loadY <= savedHeight)
					goto done;
				tested |= 16;
				if (!YMGRE_DemoHost_InjectKey(&host, 'w', 0) ||
					!YMGRE_DemoHost_InjectKey(&host, 'h', 1) ||
					!YMGRE_DemoHost_InjectKey(&host, 'c', 1))
					goto done;
			}
			if (frame == 61)
			{
				if (m.hoist.hold || view != 1)
					goto done;
				tested |= 32;
				if (!YMGRE_DemoHost_InjectKey(&host, 'k', 1) ||
					!YMGRE_DemoHost_InjectKey(&host, 'g', 1))
					goto done;
			}
			if (frame == 65 && (!YMGRE_DemoHost_InjectKey(&host, 'm', 1) ||
								!YMGRE_DemoHost_InjectKey(&host, YMGRE_KEY_UP, 1)))
				goto done;
			if (frame == 70 && (!YMGRE_DemoHost_InjectKey(&host, YMGRE_KEY_DOWN, 1) ||
								!YMGRE_DemoHost_InjectKey(&host, 't', 1)))
				goto done;
			if (frame == 80 && !YMGRE_DemoHost_InjectKey(&host, YMGRE_KEY_HOME, 1))
				goto done;
			if (frame == 81)
			{
				if (m.time > 1.0 / 60 + .001 || !m.hoist.hold || m.belt[1].mu != .85 ||
					!m.belt[0].motor)
					goto done;
				tested |= 64;
			}
			if (frame == 100 && !YMGRE_DemoHost_InjectKey(&host, YMGRE_KEY_ESCAPE, 1))
				goto done;
		}
		int a = YMGRE_DemoHost_TakeActions(&host),
			lift =
				(!!(YMGRE_DemoHost_HeldKeys(&host) & 1) - !!(YMGRE_DemoHost_HeldKeys(&host) & 2)),
			steps = 0;
		if (a & QUIT)
		{
			result = smoke ? !(changed == 15 && tested == 127) : 0;
			break;
		}
		if (a & RESET)
		{
			Mechanisms_Init(&m);
			accumulator = 0;
			elapsed = 0;
			paused = 0;
			automatic = 1;
			autoClock = 0;
		}
		if (a & PAUSE)
		{
			paused = !paused;
			elapsed = 0;
			accumulator = 0;
		}
		if (a & AUTO)
		{
			automatic = !automatic;
			autoClock = 0;
		}
		if (lift)
			automatic = 0;
		for (int i = 0; i < 2; ++i)
		{
			pme_belt *b = &m.belt[i];
			if (a & REVERSE)
			{
				b->target = -b->target;
				automatic = 0;
			}
			if (a & MOTOR)
			{
				b->motor = !b->motor;
				automatic = 0;
			}
			if (a & (UP | DOWN))
			{
				b->target = copysign(
					fmax(0, fmin(MECH_PI, fabs(b->target) + ((a & UP) ? .2 : -.2))), b->target);
				automatic = 0;
			}
		}
		if (a & FRICTION)
			m.belt[1].mu = m.belt[1].mu ? 0 : .85;
		if (a & HOLD)
		{
			m.hoist.hold = !m.hoist.hold;
			automatic = 0;
		}
		if (a & KICK)
			Swing_Kick(&m.swing);
		if (a & CAMERA)
			view = !view;
		if (a & CONTACT)
			contacts = !contacts;
		if (paused && (a & STEP) && !Mechanisms_Step(&m, lift, 1.0 / 120))
			goto done;
		if (!paused)
		{
			accumulator += fmin(elapsed, .2);
			while (accumulator >= 1.0 / 120 && steps++ < 24)
			{
				double command = lift;
				if (automatic)
				{
					command = sin(m.time);
					autoClock += 1.0 / 120;
					if (autoClock >= 5)
					{
						m.belt[0].target = -m.belt[0].target;
						m.belt[1].target = -m.belt[1].target;
						autoClock = 0;
					}
				}
				if (!Mechanisms_Step(&m, command, 1.0 / 120))
					goto done;
				accumulator -= 1.0 / 120;
			}
			if (accumulator >= 1.0 / 120)
				accumulator = fmod(accumulator, 1.0 / 120);
		}
		for (int i = 0; i < 4; ++i)
		{
			if (!MechScene_Render(&scenes[i], &m, view, contacts))
				goto done;
			YMGUI_Obj_Invalidate(images[i]);
			if (smoke)
			{
				uint64_t h = hash(YMGRE_Camera_GetRenderTarget(scenes[i].camera));
				if (!frame)
					first[i] = h;
				else if (h != first[i])
					changed |= 1 << i;
			}
		}
		if (frame % 4 == 0 || a)
		{
			char text[160];
			for (int i = 0; i < 2; ++i)
			{
				pme_belt *b = &m.belt[i];
				snprintf(text, sizeof(text), "RPM  A %+.2f   B %+.2f", b->omega[0] * 30 / MECH_PI,
						 b->omega[1] * 30 / MECH_PI);
				YMGUI_Label_SetText(stats[i][0], text);
				if (i)
					snprintf(text, sizeof(text), "Friction %.2f | Contacts %d | Slip %+.3f m/s",
							 b->mu, b->contacts,
							 b->omega[0] * b->radius[0] - b->omega[1] * b->radius[1]);
				else
					snprintf(text, sizeof(text), "No-slip ratio 0.65 / 0.95 | Same direction");
				YMGUI_Label_SetText(stats[i][1], text);
				if (i)
					snprintf(text, sizeof(text), "96 belt nodes | Stretch %.2f%% | F = friction",
							 b->maxStretch * 100);
				else
					snprintf(text, sizeof(text), "Fixed axes | Motor A / resisting load on B");
				YMGUI_Label_SetText(stats[i][2], text);
			}
			snprintf(text, sizeof(text), "Load 8 kg | Lift %+.3f m | %s", m.hoist.loadY + .35,
					 m.hoist.hold ? "HOLD" : "RELEASED");
			YMGUI_Label_SetText(stats[2][0], text);
			snprintf(text, sizeof(text), "Rope T %.2f N | Handle pull %.2f N", m.hoist.tension,
					 m.hoist.pull);
			YMGUI_Label_SetText(stats[2][1], text);
			YMGUI_Label_SetText(stats[2][2], "4 supporting strands | Pull 4 m -> lift 1 m");
			snprintf(text, sizeof(text), "Mass 2 kg | Rope %.2f m | Angle %+.1f deg",
					 m.swing.length,
					 atan2(m.swing.position.x - m.swing.anchor.x,
						   m.swing.anchor.y - m.swing.position.y) *
						 180 / MECH_PI);
			YMGUI_Label_SetText(stats[3][0], text);
			snprintf(text, sizeof(text), "Tension %.2f N | Gravity-driven free swing",
					 m.swing.tension);
			YMGUI_Label_SetText(stats[3][1], text);
			YMGUI_Label_SetText(stats[3][2], "Initial mass center = rope end | Zero velocity");
			snprintf(text, sizeof(text),
					 "%s | %s | Belt target %+.1f RPM | Gold input / Blue output",
					 paused		 ? "PAUSED"
					 : automatic ? "AUTO"
								 : "MANUAL",
					 m.belt[0].motor ? "MOTOR" : "COAST", m.belt[0].target * 30 / MECH_PI);
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
	for (int i = 0; i < 4; ++i)
		MechScene_Destroy(&scenes[i]);
	if (smoke)
		printf("%s mechanisms window: changed=%d controls=%d frame=%d\n", result ? "FAIL" : "PASS",
			   changed, tested, frame);
	return result;
}
