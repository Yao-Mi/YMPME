/* 默认打开实时窗口；--headless 保留离屏输出。 */
#include "scene.h"
#include <stdio.h>
#include <string.h>
int main(int argc, char** argv)
{
	ball_scene scene;
	int result = 1, headless = 0, smokeTest = 0;
	unsigned step;
	if (argc == 2 && strcmp(argv[1], "--headless") == 0) headless = 1;
	else if (argc == 2 && strcmp(argv[1], "--smoke-test") == 0) smokeTest = 1;
	else if (argc != 1)
	{
		fprintf(stderr, "Usage: %s [--headless|--smoke-test]\n", argv[0]);
		return 1;
	}
#ifndef YMPME_GRE_WINDOW
	if (smokeTest) { fprintf(stderr, "Window support is disabled.\n"); return 1; }
	headless = 1;
#endif
	if (!BallScene_Init(&scene))
	{
		fprintf(stderr, "Could not initialize the bouncing-ball scene.\n");
		BallScene_Destroy(&scene);
		return 1;
	}
#ifdef YMPME_GRE_WINDOW
	if (!headless)
	{
		result = BallWindow_Run(&scene, smokeTest);
		goto cleanup;
	}
#endif
	for (step = 0; step <= 240; ++step)
	{
		if (step) YMPME_World_Step(&scene.world, scene.world.config.fixedStep);
		if (step == 0 || step == 60 || step == 120 || step == 240)
		{
			if (!BallScene_Render(&scene, 1) || !BallScene_SaveFrame(&scene, step)) goto cleanup;
		}
	}
	result = 0;
cleanup:
	BallScene_Destroy(&scene);
	return result;
}
