#ifndef YMPME_DYNAMICS_H
#define YMPME_DYNAMICS_H
#include "../JOINT/YMPME_Joint.h"
#ifdef __cplusplus
extern "C"
{
#endif
	typedef enum pme_collider_type_
	{
		PME_COLLIDER_SPHERE,
		PME_COLLIDER_BOX,
		PME_COLLIDER_PLANE,
		PME_COLLIDER_MESH
	} pme_collider_type;
	/* Local shape pose is relative to the rigid body COM. Plane: shape y=0, +Y outward,
	 * static body only. Box half extents / sphere radius, metres. */
	typedef struct pme_collider_
	{
		size_t body;
		pme_collider_type type;
		pme_vec3 halfSize;
		float radius, friction, restitution;
		const pme_convex_mesh
			*mesh; /* Init-validated immutable borrowed geometry, local shape frame. */
		uint32_t category, mask;
		int32_t group;			/* equal nonzero: negative ignore, positive always collide */
		pme_vec3 localPosition; /* metres in the body COM frame */
		pme_quaternion localOrientation; /* shape -> body; zero also means identity */
	} pme_collider;
	typedef struct pme_dynamic_contact_
	{
		size_t colliderA, colliderB, bodyA, bodyB;
		pme_vec3 point, normal; /* common world point, normal A -> B */
		float penetration, normalImpulse;
		pme_vec3 tangentImpulse;
		pme_wrench reactionA, reactionB; /* N / Nm, about each START-of-step COM */
		pme_vec3 originA, originB;
		/* Fixed caller-owned solver workspace; do not edit during Step. */
		pme_joint_row rows[3];
		float friction, tangentK01;
		pme_vec3 localPointA, localPointB; /* cache anchors in body COM frames */
		unsigned persistence;			   /* consecutive matched steps; 1 for a new contact */
	} pme_dynamic_contact;
	/* Caller-owned optional cross-step contact cache. Entries are solver workspace:
	 * initialize through CacheInit, and do not edit entries during its lifetime. */
	typedef struct pme_contact_cache_entry_
	{
		size_t colliderA, colliderB, bodyA, bodyB;
		pme_vec3 localPointA, localPointB, normal, tangentImpulse;
		float normalImpulse;
		unsigned persistence, used;
	} pme_contact_cache_entry;
	typedef struct pme_contact_cache_
	{
		pme_contact_cache_entry *entries;
		size_t capacity, count, hits;
		float anchorTolerance,
			normalCosine; /* default .02 m / .95; adapt tolerance to shape scale */
		float interval;
		double time;
		const pme_rigid_body *bodies;
		const pme_collider *colliders;
		size_t bodyCount, colliderCount;
	} pme_contact_cache;
	pme_result YMPME_Dynamics_CacheInit(pme_contact_cache *cache, pme_contact_cache_entry *entries,
										size_t capacity);
	/* Clear after teleporting/resetting/reusing bodies, or changing shape, local
	 * pose, body mass/inertia or array order. Keeps the buffer and match settings. */
	void YMPME_Dynamics_CacheClear(pme_contact_cache *cache);
    /* CCD impact events are separate from start-pose discrete contacts and caches.
     * Normal points from static collider A to moving sphere B. Wrenches are impulse/dt
     * about the COMs at impact; time is seconds since this step began. */
    typedef struct pme_ccd_event_
    {
        size_t colliderA, colliderB, bodyA, bodyB;
        float time, normalImpulse;
        pme_vec3 point, normal, tangentImpulse, originA, originB;
        pme_wrench reactionA, reactionB;
    } pme_ccd_event;
    typedef struct pme_ccd_
    {
        const uint8_t *enabled; /* bodyCount flags; one centered sphere per enabled body */
        size_t bodyCount;
        pme_ccd_event *events;
        size_t capacity, count, clampedBodies;
        unsigned maxImpacts; /* 1..32 per body per step; default 8 */
    } pme_ccd;
    /* Enabled bodies must have one COM-centered sphere and no enabled joints.
     * Static plane/box/convex/sphere targets only; dynamic pairs remain discrete.
     * capacity >= enabled body count * maxImpacts, validated before mutation.
     * At impact cap, further colliding motion is frozen; inspect clampedBodies.
     * All buffers caller-owned, nonoverlapping; flags 0/1. NULL world.ccd disables. */
    pme_result YMPME_Dynamics_CCDInit(pme_ccd *ccd, const uint8_t *enabled,
                                     size_t bodyCount, pme_ccd_event *events, size_t capacity);
	typedef struct pme_dynamics_
	{
		pme_rigid_body *bodies;
		size_t bodyCount;
		pme_joint *joints;
		size_t jointCount;
		const pme_collider *colliders;
		size_t colliderCount;
		pme_dynamic_contact *contacts;
		size_t contactCapacity, contactCount;
		pme_vec3 gravity;
		float slop, stabilization, maxCorrectionSpeed, bounceThreshold;
		unsigned iterations;
		double time;
		float interval;
		pme_contact_cache *cache; /* NULL = cold contacts, no contact cache work/storage */
		pme_joint_cache *jointCache; /* NULL = cold joints; independent of contact cache */
        pme_ccd *ccd; /* NULL = discrete integration. Optional sphere/static TOI integration. */
	} pme_dynamics;
	pme_collider YMPME_Dynamics_Collider(size_t body, pme_collider_type type);
	pme_dynamics YMPME_Dynamics_Default(void);
	/* One integration per body, interleaved joint/contact impulses, 3D Coulomb disk.
	 * Sphere, oriented box and convex Mesh collide in all combinations and with static planes.
	 * Polyhedral pairs use up to four representative contacts (face clipping/edge witnesses).
	 * Fixed/fixed, same-body and collision-filtered pairs are skipped.
	 * O(n^2) AABB broad phase, discrete detection plus optional sphere/static CCD. Optional warm start.
	 * All buffers caller-owned, non-overlapping and stable during Step; no allocation.
	 * dt=(0,.05], endTime strictly advances every body, iterations=1..256.
	 * Validation/capacity failure leaves bodies, joints, both caches, CCD output and time unchanged; contact
	 * workspace may be overwritten. Capacity failure must be handled, never truncated.
	 * Feedback is total solved impulse/dt, includes stabilization. Point is the
	 * midpoint of geometric witnesses at the start pose (finite penetration error).
	 * Contact forces are not a material pressure/contact-area model.
	 * Do not also call World/Joint/Rigid/AIRCRAFT Step on these bodies. */
	pme_result YMPME_Dynamics_Step(pme_dynamics *world, float dt, double endTime);
#ifdef __cplusplus
}
#endif
#endif
