#include "YMPME_Pulley.h"
#include <string.h>
static double clamp(double x, double a, double b)
{
	return fmax(a, fmin(b, x));
}
void YMPME_Pulley_Init(pme_pulley *h)
{
	if (!h)
		return;
	memset(h, 0, sizeof(*h));
	h->loadY = -.35;
	h->freeY = .9;
	h->constant = 4 * h->loadY + h->freeY;
	h->loadMass = 8;
	h->handleMass = .5;
	h->hold = 1;
}
void YMPME_Pulley_Step(pme_pulley *h, double command, double dt)
{
	if (!h || !isfinite(dt) || dt <= 0 || dt > .01 || !isfinite(command) || command < -1 || command > 1 ||
		!isfinite(h->loadMass) || h->loadMass <= 0 || !isfinite(h->handleMass) || h->handleMass <= 0)
		return;
	double equilibrium = h->loadMass * 9.81 / 4 - h->handleMass * 9.81;
	h->pull = h->hold ? clamp(equilibrium + 18 * (h->freeV + 4 * .24 * command), 0, 65) : 0;
	h->loadV -= 9.81 * dt;
	h->freeV -= (9.81 + h->pull / h->handleMass) * dt;
	/* J = [4, 1]: project velocity onto the shared rope-length constraint. */
	double impulse = -(4 * h->loadV + h->freeV) / (16 / h->loadMass + 1 / h->handleMass);
	h->loadV += 4 * impulse / h->loadMass;
	h->freeV += impulse / h->handleMass;
	h->tension = impulse / dt;
	h->loadY += h->loadV * dt;
	if (h->loadY < -.5 || h->loadY > .3)
	{
		h->loadY = clamp(h->loadY, -.5, .3);
		h->loadV = h->freeV = 0;
	}
	h->freeY = h->constant - 4 * h->loadY;
}
