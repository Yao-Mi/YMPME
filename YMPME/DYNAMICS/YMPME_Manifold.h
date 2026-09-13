#ifndef YMPME_MANIFOLD_H
#define YMPME_MANIFOLD_H
#include "YMPME_Dynamics.h"
/* Private narrow phase. Fixed stack workspace, at most four representative
 * contacts per convex pair. Inputs reference immutable Init-validated meshes. */
typedef struct pme_manifold_point_
{
	pme_vec3 normal, pointA, pointB;
	float depth;
} pme_manifold_point;
typedef struct pme_collision_pose_
{
	pme_vec3 position;
	pme_quaternion orientation;
} pme_collision_pose;
size_t PME_ConvexManifold(const pme_collider *a, const pme_collision_pose *ba, const pme_collider *b,
						  const pme_collision_pose *bb, pme_manifold_point out[4]);
#endif
