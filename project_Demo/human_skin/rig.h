#ifndef PME_HUMAN_SKIN_RIG_H
#define PME_HUMAN_SKIN_RIG_H
#include "scene.h"
#include "uv.h"
#include "self_touch.h"
#include "ground.h"
#include "contact_view.h"
#include "force_channels.h"
#include "MODEL/SKIN/YMPME_SkinContact.h"
#include "SENSOR/TACTILE/YMPME_SurfaceTactile.h"
#include "SENSOR/FORCE/YMPME_ForceDevice.h"
typedef struct human_skin_rig_
{
	demo_scene human;
	human_self_touch self;
	human_ground ground;
	human_contact_view contactView;
	human_uv_map autoUv, sourceUv;
	float autoCorners[DEMO_FACES * 6], sourceCorners[DEMO_FACES * 6];
	uint16_t triangle[DEMO_FACES][3];
	unsigned face[DEMO_FACES];
	size_t count;
	pme_surface_patch patches[DEMO_FACES];
	pme_vec3 forces[DEMO_FACES], localForces[DEMO_FACES], previousPoint[DEMO_FACES];
	pme_surface_taxel taxels[DEMO_FACES];
	pme_surface_tactile_frame touch;
	pme_skin_probe probe;
	pme_skin_contact_desc contact;
	pme_skin_contact_result reaction;
	pme_wrench boneLoads[DEMO_BONES];
	pme_force_device loadCell;
	pme_force_reading loadReading;
	GRErgb24 colors[DEMO_FACES];
	GRE_Camera4d uvCamera, legend;
	gre_material heatMaterial;
	GRErgb24 white;
	gre_listnode heatNode;
	pme_vec3 previousProbe, normal, tangentU, tangentV;
	float uvZoom, uvCenterU, uvCenterV;
	float indentation, offsetU, offsetV, angle, gain, totalNormal, peakPressure;
	double time;
	int probePatch, selectedUv, loadBone, region, charts, source, colorMode, heat, bones, wire,
		motion, rub, arrows, relative, pickTest, contactMarks;
	size_t rgbClipped;
} human_skin_rig;
int HumanSkin_Init(human_skin_rig *s);
int HumanSkin_Region(human_skin_rig *s, int region);
int HumanSkin_Select(human_skin_rig *s, int patch);
int HumanSkin_Step(human_skin_rig *s);
int HumanSkin_Joint(human_skin_rig *s, int bone);
int HumanSkin_IsContactMarked(const human_skin_rig *s, size_t face);
void HumanSkin_Render(human_skin_rig *s);
void HumanSkin_Destroy(human_skin_rig *s);
int HumanSkin_PickUV(const human_skin_rig *s, float x, float y);
void HumanSkin_ZoomUV(human_skin_rig *s, float x, float y, float factor);
int HumanSkin_Pick3D(const human_skin_rig *s, float x, float y);
const human_uv_map *HumanSkin_UV(const human_skin_rig *s);
const char *HumanSkin_RegionName(int region);
#endif
