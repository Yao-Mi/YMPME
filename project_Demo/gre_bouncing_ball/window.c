/* 复用 GRE 的 YMGUI 窗口宿主；只在应用层连接输入、物理与渲染。 */
#include "scene.h"
#include "demo_host.h"
#include "YMGUI_Invalidate.h"
#include "YMGUI_Label.h"
#include <stdio.h>
#include <string.h>
enum
{
	ACTION_PAUSE = 1,
	ACTION_RESET = 2,
	ACTION_JUMP = 4,
	ACTION_QUIT = 8
};

static const YMGRE_DemoKeyBinding keyBindings[] = {
	{YMGRE_KEY_SPACE, 0, ACTION_PAUSE, 0},
	{'r', 0, ACTION_RESET, 0},
	{YMGRE_KEY_UP, 0, ACTION_JUMP, 0},
	{YMGRE_KEY_ESCAPE, 0, ACTION_QUIT, 0},
};

static uint64_t frameHash(GRE_RenderTarget target)
{
	const unsigned char *bytes = (const unsigned char *)target->data;
	size_t i, count = (size_t)target->width * target->height * sizeof(*target->data);
	uint64_t hash = UINT64_C(14695981039346656037);
	for (i = 0; i < count; ++i)
		hash = (hash ^ bytes[i]) * UINT64_C(1099511628211);
	return hash;
}
static GYOBJ makeLabel(GYOBJ parent, int y, const char *text)
{
	GYOBJ label = YMGUI_Creat_Label_Creat(parent, 14, y, 610, 24);
	if (label)
	{
		YMGUI_Label_SetText(label, text);
		YMGUI_Label_SetTextColor(label, GY_ARGB(255, 225, 231, 239));
	}
	return label;
}
int BallWindow_Run(ball_scene *scene, int smokeTest)
{
	YMGRE_DemoHost host;

	GYOBJ image, status;
	double previous;
	unsigned frame = 0, actionsSeen = 0;
	uint64_t pausedStep = 0, firstHash = 0;
	int paused = 0, result = 1, changed = 0;
	/* 640x500 的逻辑画布默认按 1 倍显示，用户仍可覆盖现有环境变量。 */
	YMGRE_DemoHost_DefaultScale(1);
	/* 窗口与输入后端由 GRE DemoHost 管理。 */
	if (!YMGRE_DemoHost_Init(&host, 640, 500, 40))
	{
		fprintf(stderr, "Window initialization failed: %s\n", YMGRE_DemoHost_LastError());
		return 1;
	}
	image = YMGRE_DemoHost_AddTarget(&host, YMGRE_Camera_GetRenderTarget(scene->camera), 0, 0, 640,
									 420);
	status = makeLabel(host.context->root, 426, "YMPME + YMGRE | RUNNING");
	if (!image || !status ||
		!makeLabel(host.context->root, 452, "SPACE Pause/Resume   R Reset   UP Bounce   ESC Exit"))
	{
		YMGRE_DemoHost_Destroy(&host);
		return 1;
	}
	if (!YMGRE_DemoHost_BindKeys(&host, keyBindings, sizeof keyBindings / sizeof keyBindings[0]))
		goto cleanup;

	previous = YMGRE_DemoHost_Time();
	puts("YMPME + YMGRE window: SPACE pause/resume, R reset, UP bounce, ESC exit.");
	fflush(stdout);
	for (;; ++frame)
	{
		double now = YMGRE_DemoHost_Time();
		float dt = (float)((double)(now - previous));
		int actions;
		previous = now;
		/* 冒烟测试通过GRE 输入事件入口验证控制，不依赖人工观察窗口。 */
		if (smokeTest)
		{
			dt = 1.0f / 60.0f;
			if ((frame == 10 || frame == 15) &&
				!YMGRE_DemoHost_InjectKey(&host, YMGRE_KEY_SPACE, 1))
				goto cleanup;
			if (frame == 20 && !YMGRE_DemoHost_InjectKey(&host, 'r', 1))
				goto cleanup;
			if (frame == 30 && !YMGRE_DemoHost_InjectKey(&host, YMGRE_KEY_UP, 1))
				goto cleanup;
			if (frame == 45 && !YMGRE_DemoHost_InjectKey(&host, YMGRE_KEY_ESCAPE, 1))
				goto cleanup;
		}
		actions = YMGRE_DemoHost_TakeActions(&host);
		if (actions & ACTION_QUIT)
		{
			if (smokeTest && (frame != 45 || actionsSeen != 7 || !changed))
				goto cleanup;
			result = 0;
			break;
		}
		if (actions & ACTION_PAUSE)
		{
			paused = !paused;
			pausedStep = scene->world.stepCount;
			actionsSeen |= ACTION_PAUSE;
			dt = 0;
		}
		if (actions & ACTION_RESET)
		{
			BallScene_Reset(scene);
			paused = 0;
			dt = 0;
			actionsSeen |= ACTION_RESET;
			if (smokeTest &&
				(scene->world.stepCount != 0 || scene->ballBody->state.position.y != 4 ||
				 scene->ballBody->state.velocity.y != 0))
				goto cleanup;
		}
		if (actions & ACTION_JUMP)
		{
			/* 给出固定起跳速度，避免连续按键把球加速到视野之外。 */
			float impulseY =
				(6.0f - scene->ballBody->state.velocity.y) * scene->ballBody->state.mass;
			YMPME_Body_AddImpulse(scene->ballBody, YMPME_Vec3(0, impulseY, 0));
			paused = 0;
			actionsSeen |= ACTION_JUMP;
			if (smokeTest && fabsf(scene->ballBody->state.velocity.y - 6) > 1e-5f)
				goto cleanup;
		}
		if (!paused && YMPME_World_Advance(&scene->world, dt) < 0)
			goto cleanup;
		if (smokeTest && paused && scene->world.stepCount != pausedStep)
			goto cleanup;
		if (!BallScene_Render(scene, YMPME_World_InterpolationAlpha(&scene->world)))
			goto cleanup;
		if (smokeTest)
		{
			uint64_t hash = frameHash(YMGRE_Camera_GetRenderTarget(scene->camera));
			if (frame == 0)
				firstHash = hash;
			else if (hash != firstHash)
				changed = 1;
		}
		if (frame % 6 == 0 || actions)
		{
			char text[64];
			snprintf(text, sizeof(text), "%s  y=%.2fm  vy=%+.2fm/s  t=%.1fs",
					 paused ? "PAUSED" : "RUNNING", scene->ballBody->state.position.y,
					 scene->ballBody->state.velocity.y,
					 (double)scene->world.stepCount * scene->world.config.fixedStep);
			YMGUI_Label_SetText(status, text);
		}
		/* GRE 修改了外部 framebuffer，必须显式通知保留模式 UI 重绘。 */
		YMGUI_Obj_Invalidate(image);
		if (!YMGRE_DemoHost_Step(&host, smokeTest ? 0 : 60))
		{
			result = smokeTest ? 1 : 0;
			break;
		}
	}
cleanup:

	YMGRE_DemoHost_Destroy(&host);
	if (smokeTest)
		printf("%s window: animation, pause/resume, reset, bounce, exit\n",
			   result ? "FAIL" : "PASS");
	return result;
}
