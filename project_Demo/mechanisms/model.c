#include "model.h"
#include <string.h>
static double clamp(double x,double a,double b){return fmax(a,fmin(b,x));}
static void swingState(swing_model *s)
{
	pme_rope_view v = YMPME_Rope_GetView(&s->rope);
	s->anchor = v.nodes[0].position;
	s->position = s->end = v.nodes[1].position;
	s->velocity = v.nodes[1].velocity;
	s->tension = v.links[0].tension;
}
void Swing_Init(swing_model *s)
{
	memset(s, 0, sizeof(*s));
	s->length = 2.1;
	s->mass = 2;
	pme_vec3 anchor = YMPME_Vec3(0, 1.45f, 0);
	pme_vec3 end =
		YMPME_VecAdd(anchor, YMPME_Vec3((float)(s->length * sin(.72)), (float)(-s->length * cos(.72)), 0));
	YMPME_Rope_InitSpan(&s->rope, s->nodes, 2, s->links, 1, 2, anchor, end, (float)s->length, (float)s->mass,
						1, 0);
	s->rope.damping = 0;
	swingState(s);
}
void Swing_Step(swing_model *s, double dt)
{
	YMPME_Rope_Step(&s->rope, (float)dt);
	swingState(s);
}
void Swing_Kick(swing_model *s)
{
	pme_vec3 d = YMPME_VecSub(s->position, s->anchor), t = YMPME_Vec3(-d.y, d.x, 0);
	t = YMPME_VecScale(t, (float)s->mass / sqrtf(YMPME_VecDot(t, t)));
	YMPME_Rope_AddImpulse(&s->rope, 1, t);
	swingState(s);
}
void Mechanisms_Init(mechanisms_model *m)
{
	memset(m, 0, sizeof(*m));
	YMPME_Belt_Init(&m->belt[0], 0);
	YMPME_Belt_Init(&m->belt[1], 1);
	YMPME_Pulley_Init(&m->hoist);
	Swing_Init(&m->swing);
}
int Mechanisms_Step(mechanisms_model *m, double command, double dt)
{
	if (!m || !isfinite(dt) || dt <= 0 || dt > .05 || !isfinite(command))
		return 0;
	int steps = (int)ceil(dt * 600);
	double h = dt / steps;
	for (int i = 0; i < steps; ++i)
	{
		YMPME_Belt_Step(&m->belt[0], h);
		YMPME_Belt_Step(&m->belt[1], h);
		YMPME_Pulley_Step(&m->hoist, clamp(command, -1, 1), h);
		Swing_Step(&m->swing, h);
	}
	m->time += dt;
	return isfinite(m->belt[1].omega[1]) && YMPME_VecFinite(m->swing.position) && isfinite(m->hoist.loadY);
}
