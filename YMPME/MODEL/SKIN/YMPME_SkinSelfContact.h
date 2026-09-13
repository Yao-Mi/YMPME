#ifndef YMPME_SKIN_SELF_CONTACT_H
#define YMPME_SKIN_SELF_CONTACT_H
#include "YMPME_SkinContact.h"
#ifdef __cplusplus
extern "C"
{
#endif
	/* World-space SI triangle soup supplied by the application, no renderer/skeleton dependency.
	 * Each region is 0..31. Region 0 may be used as disabled (mask row = 0). */
	typedef struct pme_skin_face_
	{
		pme_vec3 vertex[3];
		pme_surface_patch surface;
		uint8_t region;
	} pme_skin_face;
	typedef struct pme_skin_self_desc_
	{
		pme_skin_contact_desc material;
		float thickness, searchDistance;
		uint32_t sourceRegions, targetRegions, allow[32];
	} pme_skin_self_desc;
	typedef struct pme_skin_self_result_
	{
		pme_wrench sourceWrench, targetWrench; /* moments about the supplied reference */
		size_t contacts, candidates;
		float maxDepth;
		int saturated;
	} pme_skin_self_result;
	typedef struct pme_skin_self_target_
	{
		size_t face;
		pme_vec3 low, high;
	} pme_skin_self_target;
	pme_skin_self_desc YMPME_SkinSelfContact_DefaultDesc(void);
	/* Four samples per source triangle (centroid + corners), nearest target triangle per
	 * sample; AABB rejection then closest point. Source and target region sets MUST be disjoint.
	 * Both allow rows must enable the pair. Adjacent regions are excluded by caller's masks.
	 * Thickness is a compliant contact shell, not mesh deformation or continuous collision.
	 * forces and moments are per-face; moments are about each face's surface.position, preserving
	 * equal/opposite force AND world torque at the common contact point. Output arrays have count
	 * entries. targets is caller-owned AABB scratch with the same capacity. Arrays must not
	 * overlap. No allocation; invalid inputs leave outputs intact. */
	pme_result YMPME_SkinSelfContact_Solve(const pme_skin_self_desc *desc,
										   const pme_skin_face *faces, size_t count,
										   pme_vec3 reference, pme_vec3 *forces, pme_vec3 *moments,
										   pme_skin_self_target *targets, size_t capacity,
										   pme_skin_self_result *out);
#ifdef __cplusplus
}
#endif
#endif
