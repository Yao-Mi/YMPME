/* 最小外部内存示例：打印 1 秒自由落体的末态。 */
#include "YMPME.h"
#include <stdio.h>
int main(void)
{
	pme_world world;
	pme_body storage[1];
	pme_body_desc desc = YMPME_Body_DefaultDesc();
	PME_Body body;
	unsigned i;
	if (YMPME_World_Init(&world, storage, 1, NULL) != PME_OK) return 1;
	desc.position.y = 10;
	body = YMPME_World_AddBody(&world, &desc);
	if (!body) return 1;
	for (i = 0; i < 120; ++i) YMPME_World_Step(&world, world.config.fixedStep);
	printf("t=1s, y=%.6f m, vy=%.6f m/s\n", body->state.position.y, body->state.velocity.y);
	YMPME_World_Destroy(&world);
	return 0;
}
