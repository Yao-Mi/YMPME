#ifndef HUMAN_SELF_TOUCH_H
#define HUMAN_SELF_TOUCH_H
#include "MODEL/SKIN/YMPME_SkinSelfContact.h"
#include "scene.h"
typedef struct human_self_touch_
{
	pme_skin_self_desc desc;
	pme_skin_self_result result;
	pme_skin_face faces[DEMO_FACES];
	pme_vec3 moments[DEMO_FACES];
	pme_skin_self_target targets[DEMO_FACES];
	pme_vec3 palmLocal, anchor, desired, corrected;
	float clock, correction, ikError, handNormal, bodyNormal;
	int enabled, cycle, phase, filtered;
} human_self_touch;
struct human_skin_rig_;
int HumanSelf_Init(struct human_skin_rig_ *s);
int HumanSelf_Enable(struct human_skin_rig_ *s, int enabled);
int HumanSelf_Pose(struct human_skin_rig_ *s, pme_vec3 palm);
int HumanSelf_Contact(struct human_skin_rig_ *s);
pme_vec3 HumanSelf_Target(struct human_skin_rig_ *s);
const char *HumanSelf_Phase(const struct human_skin_rig_ *s);
#endif
