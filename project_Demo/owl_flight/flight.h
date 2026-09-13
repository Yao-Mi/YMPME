#ifndef OWL_FLIGHT_H
#define OWL_FLIGHT_H
#include "owl/scene.h"
#include "MODEL/AIRCRAFT/YMPME_Aircraft.h"
enum
{
	OWL_WORLD_OBJECTS = 25
};
typedef struct owl_flight_
{
	demo_scene visual;
	pme_aircraft aircraft;
	GRE_Object4d scenery[OWL_WORLD_OBJECTS];
	gre_listnode sceneryNodes[OWL_WORLD_OBJECTS];
	int sceneryCount, glideFrame;
	float flapBlend;
} owl_flight;
int OwlFlight_Init(owl_flight *s, const char *assetRoot);
int OwlFlight_Reset(owl_flight *s);
int OwlFlight_Step(owl_flight *s, const pme_aircraft_command *c, pme_vec3 wind);
/* One skinning pass per displayed frame: world transform enters root pose. */
int OwlFlight_Render(owl_flight *s, int view, int wire, int bones);
void OwlFlight_Destroy(owl_flight *s);
#endif
