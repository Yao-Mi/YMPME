/* 无窗口的完整游戏时钟：帧时间 -> 固定物理步 -> 插值位置。 */
#include "YMPME.h"
#include <stdio.h>
int main(void)
{
	PME_World world = YMPME_Create_World(8, NULL);
	pme_body_desc d = YMPME_Body_DefaultDesc();
	PME_Body ball;
	unsigned frame;
	if (!world) return 1;
	d.type = PME_STATIC;
	d.shape = PME_BOX;
	d.halfExtents = YMPME_Vec3(10, 0.5f, 10);
	d.position.y = -0.5f;
	d.restitution = 0.7f;
	if (!YMPME_World_AddBody(world, &d))
	{
		YMPME_Free_World(world);
		return 1;
	}
	d = YMPME_Body_DefaultDesc();
	d.position.y = 4;
	d.restitution = 0.7f;
	ball = YMPME_World_AddBody(world, &d);
	if (!ball)
	{
		YMPME_Free_World(world);
		return 1;
	}
	puts("time,y,vy,render_y");
	for (frame = 0; frame < 360; ++frame)
	{
		pme_vec3 renderPosition;
		YMPME_World_Advance(world, 1.0f/60.0f);
		renderPosition = YMPME_Body_InterpolatedPosition(ball, YMPME_World_InterpolationAlpha(world));
		if (frame % 15 == 0)
			printf("%.3f,%.6f,%.6f,%.6f\n", (frame+1)/60.0f,
				ball->state.position.y, ball->state.velocity.y, renderPosition.y);
	}
	YMPME_Free_World(world);
	return 0;
}
