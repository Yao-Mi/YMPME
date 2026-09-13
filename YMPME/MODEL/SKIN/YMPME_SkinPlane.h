#ifndef YMPME_SKIN_PLANE_H
#define YMPME_SKIN_PLANE_H
#include "YMPME_SkinContact.h"
#ifdef __cplusplus
extern "C"
{
#endif
	/* Plane's unit normal points out of the solid. Offset is dot(normal, pointOnPlane).
	 * shell (m) supplies a compliant contact layer above the geometric plane.
	 * Centroid quadrature of outward-facing skin patches, not CCD or mesh clipping.
	 * Outputs are world forces on skin; reaction is on the plane about reference.
	 * Input/output arrays must not overlap. Validated before writes, no allocation. */
	typedef struct pme_skin_plane_
	{
		pme_vec3 normal, velocity, reference;
		float offset, shell;
	} pme_skin_plane;
	pme_result YMPME_SkinContact_Plane(const pme_skin_contact_desc *desc,
									   const pme_surface_patch *patches, size_t count,
									   const pme_skin_plane *plane, pme_vec3 *forceOnSkin,
									   size_t capacity, pme_skin_contact_result *out);
#ifdef __cplusplus
}
#endif
#endif
