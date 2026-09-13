#ifndef YMPME_SOFTPAD_H
#define YMPME_SOFTPAD_H
#include "../CORE/YMPME_Surface.h"
#ifdef __cplusplus
extern "C"
{
#endif
	/* Massless viscoelastic foundation: one normal compression DOF and a tangential
	 * elastic bristle per sample. No bones, renderer, sensor, mesh or rigid solver. */
	typedef struct pme_soft_pad_desc_
	{
		float stiffness, damping; /* Pa/m, Pa s/m */
		float shearStiffness, shearDamping, friction;
		float maxCompression; /* m; backing contact must handle bottoming out */
	} pme_soft_pad_desc;
	typedef struct pme_soft_pad_sample_
	{
		int hit;
		float compression; /* Required inward travel from the undeformed sample, m. */
		pme_vec3 normal;   /* Unit obstacle outward normal, directed towards this pad. */
		pme_vec3 velocity; /* Obstacle velocity at this contact point, world m/s. */
	} pme_soft_pad_sample;
	typedef struct pme_soft_pad_frame_
	{
		pme_vec3 supportOrigin, supportAngularVelocity, otherOrigin;
	} pme_soft_pad_frame;
	typedef struct pme_soft_pad_node_
	{
		float compression, compressionSpeed;
		pme_vec3 shear, position, force; /* world; force acts on pad */
		int contact;
	} pme_soft_pad_node;
	typedef struct pme_soft_pad_result_
	{
		pme_wrench support, other; /* opposite forces, torques about frame origins */
		size_t contacts, bottomedOut;
		float maxCompression, sampledArea, elasticEnergy;
	} pme_soft_pad_result;
	pme_soft_pad_desc YMPME_SoftPad_Default(void);
	/* Zero-initialize nodes before first use. Geometry provides samples separately;
	 * NULL samples means unloading/free recovery. rest is the already-posed outer
	 * surface (e.g. after skeletal skinning), with fixed reference areas and support
	 * velocities. Do not reapply the skeletal transform to the returned positions.
	 * Backward-Euler Kelvin-Voigt recovery; unilateral contact cannot pull an object.
	 * Coulomb-bounded shear spring allows static grip before sliding. No volume mesh,
	 * tissue inertia, neighbour coupling or CCD. Feed both wrenches into your physics
	 * integrator once per substep; Step itself never advances the supporting bodies.
	 * All storage caller-owned, no allocation. Buffers must not overlap. dt=(0,.02].
	 * Invalid input / insufficient capacity does not modify nodes or result.
	 * Contact force is a constitutive sample, not an impulse-averaged sensor reading.
	 * maxCompression exhaustion is reported, not silently treated as a hard barrier. */
	pme_result YMPME_SoftPad_Step(const pme_soft_pad_desc *desc, const pme_surface_patch *rest,
								  const pme_soft_pad_sample *samples, size_t count,
								  const pme_soft_pad_frame *frame, float dt,
								  pme_soft_pad_node *nodes, size_t capacity,
								  pme_soft_pad_result *out);
#ifdef __cplusplus
}
#endif
#endif
