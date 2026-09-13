#ifndef MECHANISMS_MODEL_H
#define MECHANISMS_MODEL_H
#include "MODEL/ROPE/YMPME_Rope.h"
#include "YMPME.h"
#define MECH_PI 3.14159265358979323846
#include "MODEL/BELT/YMPME_Belt.h"
#include "MODEL/PULLEY/YMPME_Pulley.h"
typedef struct swing_model_
{
	pme_rope rope;
	pme_rope_node nodes[2];
	pme_rope_link links[1];
	pme_vec3 anchor, end, position, velocity;
	double length, mass, tension;
} swing_model;
typedef struct mechanisms_model_
{
	pme_belt belt[2];
	pme_pulley hoist;
	swing_model swing;
	double time;
} mechanisms_model;
void Swing_Init(swing_model *s);
void Swing_Step(swing_model *s, double dt);
void Swing_Kick(swing_model *s);
void Mechanisms_Init(mechanisms_model *m);
int Mechanisms_Step(mechanisms_model *m, double hoistCommand, double dt);
#endif
