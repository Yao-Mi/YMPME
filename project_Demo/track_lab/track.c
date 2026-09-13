#include "track.h"
#include <string.h>
#define V YMPME_Vec3
#define PI 3.14159265358979323846f
static pme_quaternion rotation(float a) {
  pme_quaternion q;
  YMPME_Quaternion_AxisAngle(V(0, 0, 1), a, &q);
  return q;
}
static int body(track_lab *s, int i, float mass, pme_vec3 size,
                pme_vec3 position, pme_quaternion q) {
  return YMPME_Rigid_Init(&s->bodies[i], mass,
                          YMPME_Rigid_BoxInertia(mass, size), position,
                          q) == PME_OK;
}
int Track_Init(track_lab *s) {
  memset(s, 0, sizeof(*s));
  s->pitch = .16f;
  s->halfSpan = 6 * s->pitch;
  float r = s->pitch / (2 * sinf(PI / 16));
  s->radius = r * cosf(PI / 16) - .035f - .001f;
  s->speed = 1.5f;
  s->friction = 1.1f;
  s->preload = 50;
  s->motor = 1;
  pme_vec3 nodes[TRACK_LINKS];
  /* Equal chord lengths, shared hinge points, clockwise closed stadium. */
  for (int i = 0; i < 12; ++i)
    nodes[i] = V(-s->halfSpan + i * s->pitch, .85f + r, 0);
  for (int i = 0; i < 8; ++i) {
    float a = PI / 2 - i * PI / 8;
    nodes[12 + i] = V(s->halfSpan + r * cosf(a), .85f + r * sinf(a), 0);
  }
  for (int i = 0; i < 12; ++i)
    nodes[20 + i] = V(s->halfSpan - i * s->pitch, .85f - r, 0);
  for (int i = 0; i < 8; ++i) {
    float a = -PI / 2 - i * PI / 8;
    nodes[32 + i] = V(-s->halfSpan + r * cosf(a), .85f + r * sinf(a), 0);
  }
  for (int i = 0; i < TRACK_LINKS; ++i) {
    pme_vec3 a = nodes[i], b = nodes[(i + 1) % TRACK_LINKS],
             d = YMPME_VecSub(b, a);
    if (!body(s, i, .12f, V(s->pitch * .94f, .07f, .24f),
              YMPME_VecScale(YMPME_VecAdd(a, b), .5f),
              rotation(atan2f(d.y, d.x))))
      return 0;
    s->colliders[i] = YMPME_Dynamics_Collider(i, PME_COLLIDER_BOX);
    s->colliders[i].halfSize = V(s->pitch * .47f, .035f, .12f);
    /* Bench excludes link/link self contact; wheels and ground still collide.
     */
    s->colliders[i].category = 1;
    s->colliders[i].mask = 2 | 4;
  }
  for (int i = 40; i < 42; ++i) {
    if (!body(s, i, 1.5f, V(2 * s->radius, 2 * s->radius, .18f),
              V(i == 40 ? -s->halfSpan : s->halfSpan, .85f, 0), rotation(0)))
      return 0;
    /* Uniform cylindrical wheel inertia; circular proxy is only contact
     * geometry. */
    float xy = s->radius * s->radius / 4;
    s->bodies[i].inertia = V(1.5f * (xy + .18f * .18f / 12),
                             1.5f * (xy + .18f * .18f / 12), 3 * xy);
    s->bodies[i].inverseInertia =
        V(1 / s->bodies[i].inertia.x, 1 / s->bodies[i].inertia.y,
          1 / s->bodies[i].inertia.z);
    s->colliders[i] = YMPME_Dynamics_Collider(i, PME_COLLIDER_SPHERE);
    s->colliders[i].radius =
        s->radius; /* Circular contact proxy for this planar bench. */
    s->colliders[i].category = 2;
    s->colliders[i].mask = 1;
  }
  if (!body(s, 42, .5f, V(.15f, .15f, .15f), V(s->halfSpan, .85f, 0),
            rotation(0)) ||
      !body(s, 43, 0, V(1, 1, 1), V(0, 0, 0), rotation(0)))
    return 0;
  s->colliders[42] = YMPME_Dynamics_Collider(43, PME_COLLIDER_PLANE);
  s->colliders[42].category = 4;
  s->colliders[42].mask = 1;
  /* Frictionless side guides constrain the bench laterally by real contact.
   * They prevent a free belt from walking off the circular wheel proxies. */
  for (int i = 0; i < 2; ++i) {
    pme_quaternion q;
    YMPME_Quaternion_AxisAngle(V(1, 0, 0), i ? -PI / 2 : PI / 2, &q);
    if (!body(s, 44 + i, 0, V(1, 1, 1), V(0, 0, i ? .122f : -.122f), q))
      return 0;
    s->colliders[43 + i] = YMPME_Dynamics_Collider(44 + i, PME_COLLIDER_PLANE);
    s->colliders[43 + i].category = 4;
    s->colliders[43 + i].mask = 1;
    s->colliders[43 + i].friction = 0;
  }
  for (int i = 0; i < TRACK_LINKS; ++i)
    if (YMPME_Joint_Init(&s->joints[i], PME_JOINT_REVOLUTE, s->bodies,
                         TRACK_BODIES, i, (i + 1) % TRACK_LINKS,
                         nodes[(i + 1) % TRACK_LINKS], rotation(0)) != PME_OK)
      return 0;
  if (YMPME_Joint_Init(&s->joints[40], PME_JOINT_REVOLUTE, s->bodies,
                       TRACK_BODIES, PME_JOINT_WORLD, 40,
                       s->bodies[40].position, rotation(0)) != PME_OK ||
      YMPME_Joint_Init(&s->joints[41], PME_JOINT_REVOLUTE, s->bodies,
                       TRACK_BODIES, 42, 41, s->bodies[41].position,
                       rotation(0)) != PME_OK)
    return 0;
  pme_quaternion slider;
  YMPME_Quaternion_AxisAngle(V(0, 1, 0), PI / 2, &slider);
  if (YMPME_Joint_Init(&s->joints[42], PME_JOINT_PRISMATIC, s->bodies,
                       TRACK_BODIES, PME_JOINT_WORLD, 42,
                       s->bodies[42].position, slider) != PME_OK ||
      YMPME_Joint_SetLimit(&s->joints[42],
                           (pme_joint_limit){1, -.035f, .075f}) != PME_OK)
    return 0;
  s->world = YMPME_Dynamics_Default();
  s->world.bodies = s->bodies;
  s->world.bodyCount = TRACK_BODIES;
  s->world.joints = s->joints;
  s->world.jointCount = TRACK_JOINTS;
  s->world.colliders = s->colliders;
  s->world.colliderCount = TRACK_COLLIDERS;
  s->world.contacts = s->contacts;
  s->world.contactCapacity = TRACK_CONTACTS;
  s->world.iterations = 64;
  s->world.maxCorrectionSpeed = .3f;
  return 1;
}
int Track_Step(track_lab *s) {
  /* Spring preload acts on the physical slider, never overwrites poses. */
  float x = s->bodies[42].position.x - s->halfSpan;
  s->tension = s->preload - 350 * x - 12 * s->bodies[42].velocity.x;
  s->bodies[42].load.force.x += s->tension;
  float desired = s->world.time < .6 ? 0 : s->speed;
  s->appliedSpeed +=
      YMPME_Clamp(desired - s->appliedSpeed, -2 * TRACK_DT, 2 * TRACK_DT);
  if (YMPME_Joint_SetMotor(&s->joints[40],
                           (pme_joint_motor){s->motor ? PME_JOINT_MOTOR_SPEED
                                                      : PME_JOINT_MOTOR_OFF,
                                             s->appliedSpeed, 0, 2}) != PME_OK)
    return 0;
  for (int i = 0; i < 43; ++i)
    s->colliders[i].friction = s->friction;
  s->status = YMPME_Dynamics_Step(&s->world, TRACK_DT,
                                  s->world.time + (double)TRACK_DT);
  if (s->status != PME_OK)
    return 0;
  s->anchorError = 0;
  s->penetration = 0;
  s->beltSpeed = 0;
  for (int i = 0; i < TRACK_JOINTS; ++i)
    s->anchorError = fmaxf(s->anchorError, s->joints[i].feedback.anchorError);
  for (size_t i = 0; i < s->world.contactCount; ++i)
    s->penetration = fmaxf(s->penetration, s->contacts[i].penetration);
  for (int i = 0; i < TRACK_LINKS; ++i) {
    pme_vec3 v = s->bodies[i].velocity;
    s->beltSpeed += sqrtf(YMPME_VecDot(v, v)) / TRACK_LINKS;
    if (!YMPME_Rigid_IsValid(&s->bodies[i]) ||
        fabsf(s->bodies[i].position.z) > .2f)
      return 0;
  }
  ++s->steps;
  return 1;
}
