#ifndef YMPME_SOFTBALL_H
#define YMPME_SOFTBALL_H
#include "../CORE/YMPME_MathBase.h"
#ifdef __cplusplus
extern "C"
{
#endif
	/* One volume-preserving affine mode, not a volumetric/FEM body. Body-local X
	 * compresses by q, Y/Z expand together. Translational/rotational dynamics and
	 * collision geometry belong to the application. No allocation or rendering. */
	typedef struct pme_soft_ball_
	{
		float radius, stiffness, damping; /* m, N/m, N s/m */
		float compression, speed;		  /* q in m, qdot in m/s */
	} pme_soft_ball;
	pme_result YMPME_SoftBall_Init(pme_soft_ball *ball, float radius, float stiffness,
								   float damping);
	/* generalizedForce = sum dot(local contact force, Mode(local contact point)).
	 * Backward Euler massless Kelvin-Voigt response. q constrained to [-.25R,.55R].
	 * Invalid input leaves state unchanged; fixed step (0,.02] s. */
	pme_result YMPME_SoftBall_Step(pme_soft_ball *ball, float generalizedForce, float dt);
	pme_vec3 YMPME_SoftBall_Scale(const pme_soft_ball *ball);
	/* Shape derivative dx/dq at a CURRENT deformed body-local point. Also converts
	 * mode speed into local surface velocity. Requires valid initialized ball. */
	pme_vec3 YMPME_SoftBall_Mode(const pme_soft_ball *ball, pme_vec3 localPoint);
#ifdef __cplusplus
}
#endif
#endif
