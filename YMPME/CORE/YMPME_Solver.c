#include "YMPME_Solver.h"
#include "YMPME_MathBase.h"
static pme_vec3 velocity(const pme_body *b)
{
	return b->state.type == PME_STATIC ? YMPME_Vec3(0, 0, 0) : b->state.velocity;
}
static void applyPair(PME_Body a, PME_Body b, pme_vec3 impulse)
{
	a->state.velocity = YMPME_VecSub(a->state.velocity, YMPME_VecScale(impulse, a->inverseMass));
	b->state.velocity = YMPME_VecAdd(b->state.velocity, YMPME_VecScale(impulse, b->inverseMass));
}
void YMPME_Solver_ResolveFeedback(PME_Body a, PME_Body b, const pme_contact *c,
								  const pme_world_desc *cfg, pme_contact_feedback *feedback)
{
	float inv = a->inverseMass + b->inverseMass;
	pme_vec3 relative, correction;
	float normalSpeed, j, restitution;
	if (a->state.sensor || b->state.sensor || inv <= 0)
		return;
	correction = YMPME_VecScale(c->normal, fmaxf(c->penetration - cfg->penetrationSlop, 0) *
											   cfg->correctionRate / inv);
	a->state.position = YMPME_VecSub(a->state.position, YMPME_VecScale(correction, a->inverseMass));
	b->state.position = YMPME_VecAdd(b->state.position, YMPME_VecScale(correction, b->inverseMass));
	relative = YMPME_VecSub(velocity(b), velocity(a));
	normalSpeed = YMPME_VecDot(relative, c->normal);
	if (normalSpeed >= 0)
		return;
	/* 低速接触关闭反弹，减少重力下的静止抖动。 */
	restitution =
		-normalSpeed > cfg->bounceThreshold ? fminf(a->state.restitution, b->state.restitution) : 0;
	j = -(1 + restitution) * normalSpeed / inv;
	applyPair(a, b, YMPME_VecScale(c->normal, j));
	if (feedback)
		feedback->normalImpulse = YMPME_VecScale(c->normal, j);
	relative = YMPME_VecSub(velocity(b), velocity(a));
	{
		pme_vec3 tangent =
			YMPME_VecSub(relative, YMPME_VecScale(c->normal, YMPME_VecDot(relative, c->normal)));
		float length = sqrtf(YMPME_VecDot(tangent, tangent));
		if (length > 1e-8f)
		{
			float mu = sqrtf(a->state.friction) * sqrtf(b->state.friction);
			float jt = fminf(length / inv, mu * j);
			applyPair(a, b, YMPME_VecScale(tangent, -jt / length));
			if (feedback)
				feedback->tangentImpulse = YMPME_VecScale(tangent, -jt / length);
		}
	}
}

void YMPME_Solver_Resolve(PME_Body a, PME_Body b, const pme_contact *c, const pme_world_desc *cfg)
{
	YMPME_Solver_ResolveFeedback(a, b, c, cfg, NULL);
}
