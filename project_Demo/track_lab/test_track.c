#include "track.h"
#include <math.h>
#include <stdio.h>
#include <time.h>
#include <string.h>
#define CHECK(x)                                                               \
  do {                                                                         \
    if (!(x)) {                                                                \
      fprintf(stderr, "line %d: %s\n", __LINE__, #x);                          \
      return 1;                                                                \
    }                                                                          \
  } while (0)
static int advance(track_lab *s, int steps, float *error, float *depth) {
  for (int i = 0; i < steps; ++i) {
    CHECK(Track_Step(s));
    *error = fmaxf(*error, s->anchorError);
    *depth = fmaxf(*depth, s->penetration);
    CHECK(s->anchorError < .005f);
    CHECK(s->penetration < .006f);
    for (int j = 0; j < TRACK_BODIES; ++j)
      CHECK(YMPME_Rigid_IsValid(&s->bodies[j]));
    for (int j = 0; j < TRACK_LINKS; ++j)
      CHECK(fabsf(s->bodies[j].position.z) < .008f);
  }
  return 0;
}
static int warm, jointWarm;
static pme_joint_cache jointCache;
static pme_joint_cache_entry jointEntries[TRACK_JOINTS];
static pme_contact_cache cache;
static pme_contact_cache_entry entries[TRACK_CONTACTS];
static int init(track_lab *s) {
 if(!Track_Init(s))return 0;
 if(warm){if(YMPME_Dynamics_CacheInit(&cache,entries,TRACK_CONTACTS)!=PME_OK)return 0;s->world.cache=&cache;}
 if(jointWarm){if(YMPME_Joint_CacheInit(&jointCache,jointEntries,TRACK_JOINTS)!=PME_OK)return 0;s->world.jointCache=&jointCache;}
 return 1;
}
int main(int argc,char **argv) {
 jointWarm=argc==2&&!strcmp(argv[1],"--warm-all");
 warm=jointWarm||(argc==2&&!strcmp(argv[1],"--warm-start"));
 if(argc>1&&!warm){fprintf(stderr,"Usage: %s [--warm-start|--warm-all]\n",argv[0]);return 1;}
 printf("contact cache: %s, joint cache: %s\n",warm?"ON":"OFF",jointWarm?"ON":"OFF");
  static track_lab s;
  float error = 0, depth = 0;
  CHECK(init(&s));
  clock_t start = clock();
  CHECK(!advance(&s, 4800, &error, &depth));
  double ms = 1000. * (clock() - start) / CLOCKS_PER_SEC / 4800;
  printf("forward: belt %.4f m/s, driver %.3f, follower %.3f rad/s, %.3f "
         "ms/step\n",
         s.beltSpeed, s.bodies[40].angularVelocity.z,
         s.bodies[41].angularVelocity.z, ms);
  CHECK(s.beltSpeed > .3f);
  CHECK(s.bodies[41].angularVelocity.z > .5f);
  CHECK(fabsf(s.joints[40].feedback.motorEffort) <= 2.001f);
  CHECK(s.bodies[42].position.x >
        s.halfSpan - .034f); /* Tensioner is not stuck at its stop. */
  s.speed = -1.5f;
  CHECK(!advance(&s, 2880, &error, &depth));
  printf("reverse: belt %.4f m/s, follower %.3f rad/s\n", s.beltSpeed,
         s.bodies[41].angularVelocity.z);
  CHECK(s.bodies[40].angularVelocity.z < -.5f);
  CHECK(s.bodies[41].angularVelocity.z < -.5f);
  s.preload = 80;
  CHECK(!advance(&s, 1440, &error, &depth));
  CHECK(s.bodies[42].position.x < s.halfSpan + .076f);
  s.motor = 0;
  CHECK(!advance(&s, 480, &error, &depth));
  CHECK(fabsf(s.joints[40].feedback.motorEffort) < 1e-6f);
  CHECK(init(&s));
  s.friction = 0;
  CHECK(!advance(&s, 4800, &error, &depth));
  printf("frictionless: belt residual sway %.4f m/s, driver %.3f, follower "
         "%.6f rad/s\n",
         s.beltSpeed, s.bodies[40].angularVelocity.z,
         s.bodies[41].angularVelocity.z);
  CHECK(s.bodies[40].angularVelocity.z > 1);
  CHECK(fabsf(s.bodies[41].angularVelocity.z) < .01f);
  CHECK(s.beltSpeed < .2f);
  printf("TRACK PASS: closed loop, reversal, tensioner, motor off, friction "
         "control; max hinge error %.3f mm, penetration %.3f mm\n",
         error * 1000, depth * 1000);
  return 0;
}
