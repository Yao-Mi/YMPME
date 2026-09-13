/* A falling sphere settles on a plane; read the actual contact force. */
#include "YMPME.h"
#include <stdio.h>
#if !PME_ENABLE_DYNAMICS || !PME_ENABLE_JOINT || !PME_ENABLE_RIGID
#error "YMPME::dynamics must propagate its feature definitions and dependencies"
#endif
#if !defined(PME_SDK_UNIFIED) && (PME_MODEL_SKIN || PME_SENSOR_TACTILE || PME_ENABLE_FLIGHT)
#error "Dynamics must not implicitly enable skin, sensors or flight"
#endif
int main(void)
{
    pme_rigid_body bodies[2];
    for (int i = 0; i < 2; ++i) {
        if (YMPME_Rigid_Init(&bodies[i], (float)i, YMPME_Vec3(.1f,.1f,.1f),
            YMPME_Vec3(0, i ? 2 : 0, 0), YMPME_Quaternion_Identity()) != PME_OK) return 1;
    }
    pme_collider shapes[2] = {YMPME_Dynamics_Collider(0, PME_COLLIDER_PLANE),
                             YMPME_Dynamics_Collider(1, PME_COLLIDER_SPHERE)};
    shapes[1].radius = .5f;
    shapes[0].restitution = shapes[1].restitution = 0;
    pme_dynamic_contact contacts[16];
    pme_dynamics world = YMPME_Dynamics_Default();
    world.bodies = bodies; world.bodyCount = 2;
    world.colliders = shapes; world.colliderCount = 2;
    world.contacts = contacts; world.contactCapacity = 16;
    for (int i = 0; i < 720; ++i) {
        if (YMPME_Dynamics_Step(&world, 1.f / 240, world.time + 1. / 240) != PME_OK) return 1;
    }
    float support = 0;
    for (size_t i = 0; i < world.contactCount; ++i)
        support += contacts[i].bodyB == 1 ? contacts[i].reactionB.force.y : contacts[i].reactionA.force.y;
    printf("Sphere: y=%.5f m, support=%.5f N\n", bodies[1].position.y, support);
    if (fabsf(bodies[1].position.y - .5f) > .005f || fabsf(support - 9.81f) > .2f) return 1;
    puts("PASS: rigid contact and force feedback without a sensor module");
    return 0;
}
