#include "scene.h"
#include "demo_host.h"
#include "YMGUI_Invalidate.h"
#include "YMGUI_Label.h"
#include <stdio.h>
#include <string.h>
enum
{
	KEY_FORWARD = 1,
	KEY_BACK = 2,
	KEY_LEFT = 4,
	KEY_RIGHT = 8,
	KEY_BRAKE = 16,
	KEY_FAST = 32
};
enum
{
	ACT_RESET = 1,
	ACT_PAUSE = 2,
	ACT_CAMERA = 4,
	ACT_QUIT = 8,
	ACT_PRECISION = 16
};

static const YMGRE_DemoKeyBinding keyBindings[] = {
	{'w', KEY_FORWARD, 0, 0},
	{'s', KEY_BACK, 0, 0},
	{'a', KEY_LEFT, 0, 0},
	{'d', KEY_RIGHT, 0, 0},
	{YMGRE_KEY_SPACE, KEY_BRAKE, 0, 0},
	{YMGRE_KEY_LSHIFT, KEY_FAST, 0, 0},
	{'r', 0, ACT_RESET, 0},
	{'p', 0, ACT_PAUSE, 0},
	{'c', 0, ACT_CAMERA, 0},
	{YMGRE_KEY_ESCAPE, 0, ACT_QUIT, 0},
	{'h', 0, ACT_PRECISION, 0},
};

static GYOBJ label(GYOBJ parent, int y, const char *text)
{
	GYOBJ obj = YMGUI_Creat_Label_Creat(parent, 10, y, 780, 22);
	if (obj)
	{
		YMGUI_Label_SetText(obj, text);
		YMGUI_Label_SetTextColor(obj, GY_ARGB(255, 224, 234, 242));
	}
	return obj;
}
static uint64_t hashFrame(GRE_RenderTarget target)
{
	uint64_t h = UINT64_C(14695981039346656037);
	const unsigned char *p = (const unsigned char *)target->data;
	size_t i, n = (size_t)target->width * target->height * sizeof(*target->data);
	for (i = 0; i < n; ++i)
		h = (h ^ p[i]) * UINT64_C(1099511628211);
	return h;
}
int main(int argc, char **argv)
{
	car_scene scene;
	YMGRE_DemoHost host;

	GYOBJ image, telemetry, score, collisionStats;
	double previous;
	double accumulator = 0;
	unsigned frame = 0;
	uint64_t firstHash = 0, pausedStep = 0;
	float brakeX = 0, brakeZ = 0;
	int paused = 0, cameraMode = 0, smoke = 0, changed = 0, result = 1, ready = 0, titleSet = 0;
	const float fixedStep = 1.0f / 120.0f;
	if (argc == 2 && strcmp(argv[1], "--smoke-test") == 0)
		smoke = 1;
	else if (argc != 1)
	{
		fprintf(stderr, "Usage: %s [--smoke-test]\n", argv[0]);
		return 1;
	}
	if (!CarScene_Init(&scene))
		goto cleanup;
	YMGRE_DemoHost_DefaultScale(1);
	/* 窗口与输入后端由 GRE DemoHost 管理。 */
	if (!YMGRE_DemoHost_Init(&host, 800, 650, 40))
	{
		fprintf(stderr, "Window initialization failed: %s\n", YMGRE_DemoHost_LastError());
		goto cleanup;
	}
	ready = 1;
	image =
		YMGRE_DemoHost_AddTarget(&host, YMGRE_Camera_GetRenderTarget(scene.camera), 0, 0, 800, 520);
	telemetry = label(host.context->root, 521, "YMPME THREE-WHEEL ROVER");
	score = label(host.context->root, 543, "Collect all 5 green pads. C switches camera.");
	collisionStats = label(host.context->root, 565, "Collision: filter -> AABB -> convex mesh");
	if (!image || !telemetry || !score || !collisionStats ||
		!label(host.context->root, 587,
			   "W/S Drive   A/D Steer / Spin   SPACE Brake   LSHIFT Boost") ||
		!label(host.context->root, 609,
			   "H Mesh/AABB+Reset   R Reset   P Pause   C Camera   ESC Exit"))
		goto cleanup;
	if (!YMGRE_DemoHost_BindKeys(&host, keyBindings, sizeof keyBindings / sizeof keyBindings[0]))
		goto cleanup;
	previous = YMGRE_DemoHost_Time();
	puts("Three-wheel rover: W/S drive, A/D steer, SPACE brake, LSHIFT boost, R reset, P pause, C "
		 "camera, H Mesh/AABB (resets), ESC exit.");
	fflush(stdout);
	for (;; ++frame)
	{
		double now = YMGRE_DemoHost_Time();
		double dt = (double)(now - previous);
		int actions, held, substeps = 0;
		float throttle, steer, left, right, maxSpeed, scale;
		previous = now;
		if (!titleSet)
		{

			if (host.context)
			{
				YMGRE_DemoHost_SetTitle(&host, "YMPME + YMGRE | Three-wheel differential rover");
				titleSet = 1;
			}
		}
		if (smoke)
		{
			dt = 1.0 / 60.0;
			if (frame == 0 && !YMGRE_DemoHost_InjectKey(&host, 'w', 1))
				goto cleanup;
			if (frame == 30)
			{
				if (scene.car.z < 0.3f || fabsf(scene.car.x) > 0.0001f)
					goto cleanup;
				if (!YMGRE_DemoHost_InjectKey(&host, 'w', 0) ||
					!YMGRE_DemoHost_InjectKey(&host, 'd', 1))
					goto cleanup;
			}
			if (frame == 60)
			{
				if (scene.car.yaw < 0.3f || fabsf(scene.car.casterAngle) < 0.1f)
					goto cleanup;
				if (!YMGRE_DemoHost_InjectKey(&host, 'd', 0) ||
					!YMGRE_DemoHost_InjectKey(&host, YMGRE_KEY_SPACE, 1))
					goto cleanup;
				brakeX = scene.car.x;
				brakeZ = scene.car.z;
			}
			if (frame == 70)
			{
				if (fabsf(scene.car.x - brakeX) + fabsf(scene.car.z - brakeZ) > 0.0001f)
					goto cleanup;
				if (!YMGRE_DemoHost_InjectKey(&host, YMGRE_KEY_SPACE, 0) ||
					!YMGRE_DemoHost_InjectKey(&host, 'p', 1))
					goto cleanup;
				pausedStep = scene.world.stepCount;
			}
			if (frame == 80)
			{
				if (scene.world.stepCount != pausedStep || !YMGRE_DemoHost_InjectKey(&host, 'p', 1))
					goto cleanup;
			}
			if (frame == 90 && !YMGRE_DemoHost_InjectKey(&host, 'r', 1))
				goto cleanup;
			if (frame == 95 && !YMGRE_DemoHost_InjectKey(&host, 'h', 1))
				goto cleanup;
			if (frame == 100 && !YMGRE_DemoHost_InjectKey(&host, 'c', 1))
				goto cleanup;
			if (frame == 110 && !YMGRE_DemoHost_InjectKey(&host, YMGRE_KEY_ESCAPE, 1))
				goto cleanup;
		}
		actions = YMGRE_DemoHost_TakeActions(&host);
		held = YMGRE_DemoHost_HeldKeys(&host);
		if (actions & ACT_QUIT)
		{
			result = smoke ? !(changed && frame == 110 && cameraMode == 1 &&
							   scene.world.config.meshPrecision == 0)
						   : 0;
			break;
		}
		if (actions & ACT_RESET)
		{
			CarScene_Reset(&scene);
			accumulator = 0;
			dt = 0;
			paused = 0;
			if (smoke && (scene.car.x != 0 || scene.car.z != 0 || scene.car.yaw != 0 ||
						  scene.world.stepCount != 0))
				goto cleanup;
		}
		if (actions & ACT_PRECISION)
		{
			CarScene_SetMeshPrecision(&scene, !scene.world.config.meshPrecision);
			accumulator = 0;
			dt = 0;
			paused = 0;
		}
		if (actions & ACT_PAUSE)
		{
			paused = !paused;
			dt = 0;
		}
		if (actions & ACT_CAMERA)
			cameraMode = !cameraMode;
		throttle = ((held & KEY_FORWARD) ? 1.0f : 0) - ((held & KEY_BACK) ? 1.0f : 0);
		steer = ((held & KEY_RIGHT) ? 1.0f : 0) - ((held & KEY_LEFT) ? 1.0f : 0);
		left = throttle + steer;
		right = throttle - steer;
		scale = fmaxf(1, fmaxf(fabsf(left), fabsf(right)));
		maxSpeed = (held & KEY_FAST) ? 3.0f : 1.6f;
		left = left * maxSpeed / scale;
		right = right * maxSpeed / scale;
		if (!paused)
		{
			/* 帧卡顿最多追赶 8 个物理步，避免界面失去响应。 */
			accumulator += fmin(dt, 0.25);
			while (accumulator >= fixedStep && substeps < 8)
			{
				if (!CarScene_Step(&scene, left, right, held & KEY_BRAKE, fixedStep))
					goto cleanup;
				accumulator -= fixedStep;
				++substeps;
			}
			if (accumulator >= fixedStep)
				accumulator = fmod(accumulator, fixedStep);
		}
		CarScene_Render(&scene, cameraMode);
		if (smoke)
		{
			uint64_t h = hashFrame(YMGRE_Camera_GetRenderTarget(scene.camera));
			if (frame == 0)
				firstHash = h;
			else if (h != firstHash)
				changed = 1;
		}
		if (frame % 6 == 0 || actions)
		{
			char text[64];
			snprintf(text, sizeof(text), "%s  L %+.2f  R %+.2f m/s   Caster %+.0f deg",
					 paused ? "PAUSED" : "DRIVE", scene.car.left, scene.car.right,
					 scene.car.casterAngle * 57.29578f);
			YMGUI_Label_SetText(telemetry, text);
			snprintf(text, sizeof(text), "%s | %s %d/5   Contacts %d",
					 scene.world.config.meshPrecision ? "MESH" : "AABB",
					 scene.targets == 5 ? "COMPLETE! Pads" : "Pads", scene.targets, scene.hits);
			YMGUI_Label_SetText(score, text);
			snprintf(text, sizeof(text), "AABB %u | Reject %u | Mesh %u | Filtered %u",
					 (unsigned)scene.world.collisionStats.aabbTests,
					 (unsigned)scene.world.collisionStats.aabbRejected,
					 (unsigned)scene.world.collisionStats.meshTests,
					 (unsigned)scene.world.collisionStats.filteredPairs);
			YMGUI_Label_SetText(collisionStats, text);
		}
		YMGUI_Obj_Invalidate(image);
		if (!YMGRE_DemoHost_Step(&host, smoke ? 0 : 60))
		{
			result = smoke ? 1 : 0;
			break;
		}
	}
cleanup:

	if (ready)
		YMGRE_DemoHost_Destroy(&host);
	CarScene_Destroy(&scene);
	if (smoke)
		printf("%s rover: drive, turn, caster, brake, pause, reset, precision, camera, exit\n",
			   result ? "FAIL" : "PASS");
	return result;
}
