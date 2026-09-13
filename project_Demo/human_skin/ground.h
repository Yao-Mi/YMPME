#ifndef HUMAN_GROUND_H
#define HUMAN_GROUND_H
#include "scene.h"
#include "MODEL/SKIN/YMPME_SkinPlane.h"
typedef struct human_ground_
{
	int enabled, mode, fresh;
	float renderZ, clock, travel, floorY, lift[2], footLoad[2], footClearance[2], otherLoad;
	pme_vec3 ankles[2], footCenter[2];
	uint8_t foot[DEMO_FACES]; /* 0 other, 1 left, 2 right; fixed bind weights */
	pme_skin_plane plane;
	pme_skin_contact_desc material;
	GRE_Object4d floor;
	gre_listnode floorNode;
} human_ground;
struct human_skin_rig_;
int HumanGround_Enable(struct human_skin_rig_ *s, int mode);
int HumanGround_Pose(struct human_skin_rig_ *s);
void HumanGround_Readout(struct human_skin_rig_ *s);
void HumanGround_Draw(struct human_skin_rig_ *s);
void HumanGround_Destroy(struct human_skin_rig_ *s);
const char *HumanGround_Mode(int mode);
#endif
