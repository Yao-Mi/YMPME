#include "playground.h"
#include "demo_host.h"
#include "YMGUI_Label.h"
#include "YMGUI_Invalidate.h"
#include <stdio.h>
#include <string.h>
enum
{
	FORWARD = 1,
	BACK = 2,
	LEFT = 4,
	RIGHT = 8,
	BRAKE = 16
};
enum
{
	RESET = 1,
	PAUSE = 2,
	CAMERA = 4,
	QUIT = 8,
	PRECISION = 16,
	WIRE = 32,
	AUTO = 64
};

static const YMGRE_DemoKeyBinding keyBindings[] = {
	{'w', FORWARD, 0, 0},
	{'s', BACK, 0, 0},
	{'a', LEFT, 0, 0},
	{'d', RIGHT, 0, 0},
	{YMGRE_KEY_SPACE, BRAKE, 0, 0},
	{'r', 0, RESET, 0},
	{'p', 0, PAUSE, 0},
	{'c', 0, CAMERA, 0},
	{'h', 0, PRECISION, 0},
	{'v', 0, WIRE, 0},
	{'t', 0, AUTO, 0},
	{YMGRE_KEY_ESCAPE, 0, QUIT, 0},
	{'1', 0, 0, 1},
	{'2', 0, 0, 2},
	{'3', 0, 0, 3},
};

static GYOBJ label(GYOBJ parent, int y, const char *text)
{
	GYOBJ o = YMGUI_Creat_Label_Creat(parent, 12, y, 876, 22);
	if (o)
	{
		YMGUI_Label_SetText(o, text);
		YMGUI_Label_SetTextColor(o, GY_ARGB(255, 227, 233, 239));
	}
	return o;
}
static uint64_t frameHash(GRE_RenderTarget target)
{
	const unsigned char *p = (const unsigned char *)target->data;
	size_t i, n = (size_t)target->width * target->height * sizeof(*target->data);
	uint64_t h = UINT64_C(14695981039346656037);
	for (i = 0; i < n; ++i)
		h = (h ^ p[i]) * UINT64_C(1099511628211);
	return h;
}
static int checkSceneCollisions(playground *p)
{
	PME_Body first = NULL, same = NULL, other = NULL;
	pme_body a, b;
	pme_contact contact;
	int i;
	for (i = 0; i < p->partCount; ++i)
	{
		fleet_part *m = &p->part[i];
		if (!m->collider || m->owner < 0)
			continue;
		if (m->collider->state.ignoreAssembly != (uint32_t)m->owner + 1)
			return 0;
		if (m->owner == 0)
		{
			if (!first)
				first = m->collider;
			else
				same = m->collider;
		}
		if (m->owner == 1)
			other = m->collider;
	}
	if (!first || !same || !other)
		return 0;
	a = *first;
	b = *same;
	YMPME_Body_SetPosition(&b, a.state.position);
	if (YMPME_World_Collide(&p->world, &a, &b, &contact))
		return 0;
	b = *other;
	YMPME_Body_SetPosition(&b, a.state.position);
	if (!YMPME_World_Collide(&p->world, &a, &b, &contact))
		return 0;
	/* 车辆实际驶向橙色障碍，整车回退应阻止穿越。 */
	YMPME_Vehicle_Init(&p->vehicle[2], PME_VEH_FOUR_WHEEL, 8, -6.4f, &p->terrain);
	for (i = 0; i < 500; ++i)
		if (!Playground_Step(p, 1, 0, 0, 1.0f / 120))
			return 0;
	if (!p->contacts || p->vehicle[2].drive.z > -4.6f || p->vehicle[2].drive.z < -5.5f)
		return 0;
	Playground_Reset(p);
	return 1;
}
int main(int argc, char **argv)
{
	static playground p;
	YMGRE_DemoHost host;
	GYOBJ image, info, contact, compression;
	const char *names[] = {"TRACKED", "THREE WHEEL", "FOUR WHEEL + SUSPENSION"};
	const char *heightmap = NULL;
	int i, smoke = 0, result = 1, ready = 0, paused = 0, camera = 0, wire = 0, automatic = 0,
		   autoDirection = 1, titleSet = 0;
	unsigned frame = 0, selectedMask = 0;
	double previous;
	uint64_t hash = 0, pauseStep = 0;
	int changed = 0, autoTested = 0;
	float maxCompressionDifference = 0, maxTilt = 0;
	double accumulator = 0;
	const float dt = 1.0f / 120;
	for (i = 1; i < argc; ++i)
	{
		if (!strcmp(argv[i], "--smoke-test"))
			smoke = 1;
		else if (!strcmp(argv[i], "--heightmap") && i + 1 < argc)
			heightmap = argv[++i];
		else if (!strcmp(argv[i], "--export-heightmap") && i + 1 < argc)
		{
			pme_terrain terrain;
			YMPME_Terrain_InitHills(&terrain);
			return Terrain_SavePGM(&terrain, argv[++i]) ? 0 : 1;
		}
		else
		{
			fprintf(
				stderr,
				"Usage: %s [--heightmap FILE.pgm] [--smoke-test] [--export-heightmap FILE.pgm]\n",
				argv[0]);
			return 1;
		}
	}
	if (!Playground_Init(&p, heightmap))
	{
		fprintf(stderr, "Could not initialize playground or height map.\n");
		goto cleanup;
	}
	if (smoke && !checkSceneCollisions(&p))
	{
		fprintf(stderr, "Scene collision/filtering check failed.\n");
		goto cleanup;
	}
	YMGRE_DemoHost_DefaultScale(1);
	/* 窗口与输入后端由 GRE DemoHost 管理。 */
	if (!YMGRE_DemoHost_Init(&host, 900, 680, 40))
	{
		fprintf(stderr, "Window: %s\n", YMGRE_DemoHost_LastError());
		goto cleanup;
	}
	ready = 1;
	image = YMGRE_DemoHost_AddTarget(&host, YMGRE_Camera_GetRenderTarget(p.camera), 0, 0, 900, 540);
	info = label(host.context->root, 541, "VEHICLE SUSPENSION PLAYGROUND");
	contact =
		label(host.context->root, 563, "Height field + wheel contacts + spring / damper forces");
	compression = label(host.context->root, 585, "Spring compression");
	if (!image || !info || !contact || !compression ||
		!label(host.context->root, 607, "1 Tracks   2 Three wheels   3 Four wheels + suspension") ||
		!label(host.context->root, 629, "W/S Drive   A/D Steer   SPACE Brake   T Auto hill test") ||
		!label(host.context->root, 651,
			   "C Camera   V Wire   H Mesh/AABB   P Pause   R Reset   ESC Exit"))
		goto cleanup;
	if (!YMGRE_DemoHost_BindKeys(&host, keyBindings, sizeof keyBindings / sizeof keyBindings[0]))
		goto cleanup;
	previous = YMGRE_DemoHost_Time();
	puts("Vehicle playground: 1/2/3 select, WASD drive, SPACE brake, T auto hill test, C camera, V "
		 "wireframe, H precision, P pause, R reset, ESC exit.");
	fflush(stdout);
	for (;; ++frame)
	{
		double now = YMGRE_DemoHost_Time();
		double frameTime = (double)(now - previous);
		int actions, held, selection, steps = 0;
		float throttle, steer;
		pme_vehicle *v;
		previous = now;
		if (!titleSet)
		{
			if (host.context)
			{
				YMGRE_DemoHost_SetTitle(
					&host, "YMPME + GRE | Three vehicles / height-field suspension test");
				titleSet = 1;
			}
		}
		if (smoke)
		{
			frameTime = 1.0 / 60;
			if (frame == 0 && (!YMGRE_DemoHost_InjectKey(&host, '1', 1) ||
							   !YMGRE_DemoHost_InjectKey(&host, 'w', 1)))
				goto cleanup;
			if (frame == 25)
			{
				if (p.vehicle[0].drive.z <= -6.25f || !YMGRE_DemoHost_InjectKey(&host, '2', 1) ||
					!YMGRE_DemoHost_InjectKey(&host, 'w', 0) ||
					!YMGRE_DemoHost_InjectKey(&host, 'd', 1))
					goto cleanup;
			}
			if (frame == 50)
			{
				if (p.vehicle[1].drive.yaw < .1f || !YMGRE_DemoHost_InjectKey(&host, '3', 1) ||
					!YMGRE_DemoHost_InjectKey(&host, 'd', 0) ||
					!YMGRE_DemoHost_InjectKey(&host, 'w', 1))
					goto cleanup;
			}
			if (frame == 200)
			{
				pauseStep = p.world.stepCount;
				if (!YMGRE_DemoHost_InjectKey(&host, 'p', 1))
					goto cleanup;
			}
			if (frame == 210)
			{
				if (p.world.stepCount != pauseStep || !YMGRE_DemoHost_InjectKey(&host, 'p', 1))
					goto cleanup;
			}
			if (frame == 215)
			{
				if (maxCompressionDifference < .02f || maxTilt < .02f ||
					!YMGRE_DemoHost_InjectKey(&host, 't', 1))
					goto cleanup;
			}
			if (frame == 225 && !YMGRE_DemoHost_InjectKey(&host, 'r', 1))
				goto cleanup;
			if (frame == 230 && !YMGRE_DemoHost_InjectKey(&host, 'h', 1))
				goto cleanup;
			if ((frame == 235 || frame == 240) && !YMGRE_DemoHost_InjectKey(&host, 'c', 1))
				goto cleanup;
			if (frame == 245 && !YMGRE_DemoHost_InjectKey(&host, YMGRE_KEY_ESCAPE, 1))
				goto cleanup;
		}
		actions = YMGRE_DemoHost_TakeActions(&host);
		held = YMGRE_DemoHost_HeldKeys(&host);
		selection = YMGRE_DemoHost_TakeValue(&host);
		if (selection)
		{
			p.selected = selection - 1;
			automatic = 0;
			selectedMask |= 1u << p.selected;
		}
		if (actions & QUIT)
		{
			result = smoke ? !(selectedMask == 7 && changed && camera == 2 && autoTested &&
							   p.world.config.meshPrecision == 0)
						   : 0;
			break;
		}
		if (actions & RESET)
		{
			Playground_Reset(&p);
			accumulator = 0;
			frameTime = 0;
			paused = 0;
			automatic = 0;
		}
		if (actions & PRECISION)
		{
			p.world.config.meshPrecision = !p.world.config.meshPrecision;
			Playground_Reset(&p);
			accumulator = 0;
			frameTime = 0;
		}
		if (actions & PAUSE)
		{
			paused = !paused;
			frameTime = 0;
		}
		if (actions & CAMERA)
			camera = (camera + 1) % 3;
		if (actions & WIRE)
			wire = !wire;
		if (actions & AUTO)
		{
			automatic = !automatic;
			autoDirection = 1;
			paused = 0;
			if (automatic)
			{
				Playground_Reset(&p);
				accumulator = 0;
				frameTime = 0;
				autoTested = 1;
			}
		}
		v = &p.vehicle[p.selected];
		throttle = ((held & FORWARD) ? 1.0f : 0) - ((held & BACK) ? 1.0f : 0);
		steer = ((held & RIGHT) ? 1.0f : 0) - ((held & LEFT) ? 1.0f : 0);
		if (automatic)
		{
			if (v->drive.z > 7.5f)
				autoDirection = -1;
			if (v->drive.z < -6.4f)
				autoDirection = 1;
			throttle = .65f * autoDirection;
			steer = 0;
			if (held & BRAKE)
				automatic = 0;
		}
		if (!paused)
		{
			accumulator += fmin(frameTime, .25);
			while (accumulator >= dt && steps < 12)
			{
				if (!Playground_Step(&p, throttle, steer, held & BRAKE, dt))
					goto cleanup;
				accumulator -= dt;
				++steps;
			}
			if (accumulator >= dt)
				accumulator = fmod(accumulator, dt);
		}
		if (smoke && p.selected == 2)
		{
			float low = 1, high = 0;
			for (i = 0; i < 4; ++i)
			{
				low = fminf(low, v->support[i].compression);
				high = fmaxf(high, v->support[i].compression);
			}
			maxCompressionDifference = fmaxf(maxCompressionDifference, high - low);
			maxTilt = fmaxf(maxTilt, fabsf(v->pitch) + fabsf(v->roll));
		}
		if (!smoke || frame % 3 == 0 || actions)
		{
			Playground_Render(&p, camera, wire);
			if (smoke)
			{
				uint64_t next = frameHash(YMGRE_Camera_GetRenderTarget(p.camera));
				if (frame == 0)
					hash = next;
				else if (hash != next)
					changed = 1;
			}
			YMGUI_Obj_Invalidate(image);
		}
		if (frame % 6 == 0 || actions || selection)
		{
			char text[64];
			snprintf(text, sizeof(text), "%s | %s | %+.2f m/s", names[p.selected],
					 paused		 ? "PAUSED"
					 : automatic ? "AUTO"
								 : "DRIVE",
					 v->speed);
			YMGUI_Label_SetText(info, text);
			snprintf(text, sizeof(text), "Pitch %+.1f  Roll %+.1f deg | Contact %d/%d | %s",
					 v->pitch * 57.29578f, v->roll * 57.29578f, v->contactCount, v->supportCount,
					 p.world.config.meshPrecision ? "MESH" : "AABB");
			YMGUI_Label_SetText(contact, text);
			if (p.selected == 2)
				snprintf(text, sizeof(text), "Spring mm: FL %.0f  FR %.0f  RL %.0f  RR %.0f",
						 v->support[0].compression * 1000, v->support[1].compression * 1000,
						 v->support[2].compression * 1000, v->support[3].compression * 1000);
			else
				snprintf(text, sizeof(text), "Suspension compression %.0f / %.0f / %.0f mm",
						 v->support[0].compression * 1000, v->support[1].compression * 1000,
						 v->support[2].compression * 1000);
			YMGUI_Label_SetText(compression, text);
		}
		if (!YMGRE_DemoHost_Step(&host, smoke ? 0 : 60))
		{
			result = smoke ? 1 : 0;
			break;
		}
	}
cleanup:

	if (ready)
		YMGRE_DemoHost_Destroy(&host);
	Playground_Destroy(&p);
	if (smoke)
		printf("%s fleet: 3 vehicles, suspension %.3fm, tilt %.3frad, input, pause, camera\n",
			   result ? "FAIL" : "PASS", maxCompressionDifference, maxTilt);
	return result;
}
