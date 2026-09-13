#ifndef PME_HAND_SKIN_RIG_H
#define PME_HAND_SKIN_RIG_H
#include "scene.h"
#include "uv_map.h"
#include "MODEL/SKIN/YMPME_SkinContact.h"
#include "SENSOR/TACTILE/YMPME_SurfaceTactile.h"
#include "SENSOR/FORCE/YMPME_ForceDevice.h"
typedef struct hand_skin_rig_
{
	demo_scene hand;
	hand_uv_map uv;
	const GRErgb24 *patchColors, *vertexColors; /* 可选三分量色图，只读显示缓存。 */
	const hand_uv_map *displayUv;				/* NULL使用资产原UV；只影响显示。 */
	GRE_Camera4d uvCamera, uvLegend;
	int uvForce, uvSelected;
	gre_material heatMaterial;
	GRErgb24 heatWhite;
	gre_listnode heatNode;
	pme_surface_patch patches[HAND_FACES];
	pme_vec3 forces[HAND_FACES];
	pme_surface_taxel taxels[HAND_FACES];
	pme_surface_tactile_frame touch;
	pme_skin_contact_desc contact;
	pme_skin_contact_result reaction;
	pme_skin_probe probe;
	pme_force_device loadCell;
	pme_force_reading loadReading;
	float weights[HAND_FACES][HAND_BONES];
	unsigned faces[HAND_FACES];
	size_t count;
	pme_vec3 anchor, normal, previousCenter;
	float vertexPressure[HAND_POINTS], vertexArea[HAND_POINTS], vertexForce[HAND_POINTS];
	int vectorHeat;
	float indentation, peakPressure, normalLoad;
	double time;
	int region, heat, bones, noise, relative;
} hand_skin_rig;
int HandSkin_Init(hand_skin_rig *s);
int HandSkin_Select(hand_skin_rig *s, int region);
int HandSkin_Step(hand_skin_rig *s);
/* Demo间共享皮肤采样/力回传/蒙皮更新；外部球的积分由调用者负责一次。 */
int HandSkin_StepSphere(hand_skin_rig *s, const pme_skin_probe *probe, pme_vec3 angularVelocity,
						pme_vec3 gravity, float motorEffort);
void HandSkin_RenderSurface(hand_skin_rig *s);
void HandSkin_Render(hand_skin_rig *s);
void HandSkin_Destroy(hand_skin_rig *s);
#endif
