#ifndef TRACK_LAB_H
#define TRACK_LAB_H
#include "YMPME.h"
#define TRACK_LINKS 40
#define TRACK_DRIVE 40
#define TRACK_IDLER 41
#define TRACK_CARRIER 42
#define TRACK_GROUND 43
#define TRACK_BODIES 46
#define TRACK_COLLIDERS 45
#define TRACK_JOINTS 43
#define TRACK_CONTACTS 1024
#define TRACK_DT (1.f / 480)
typedef struct track_lab_ {
  pme_rigid_body bodies[TRACK_BODIES];
  pme_joint joints[TRACK_JOINTS];
  pme_collider colliders[TRACK_COLLIDERS];
  pme_dynamic_contact contacts[TRACK_CONTACTS];
  pme_dynamics world;
  float radius, pitch, halfSpan, speed, friction, preload;
  float anchorError, penetration, beltSpeed, tension, appliedSpeed;
  unsigned long steps;
  pme_result status;
  int motor;
} track_lab;
int Track_Init(track_lab *s);
int Track_Step(track_lab *s);
#endif
