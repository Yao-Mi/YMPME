#include "control.h"
#include <stdio.h>
#include <stdlib.h>
#define CHECK(c)                                                                                   \
    do                                                                                             \
    {                                                                                              \
        if (!(c))                                                                                  \
        {                                                                                          \
            fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #c);                                \
            return 1;                                                                              \
        }                                                                                          \
    } while (0)
int main(void)
{
    joint_lab_state s;
    int i;
    float angle;
    CHECK(JointLab_Init(&s));
    for (i = 0; i < 2400; ++i)
        CHECK(JointLab_Step(&s));
    CHECK(fabsf(s.pendulum.joint.feedback.coordinate - .6f) < .002f);
    CHECK(s.sample.velocityValid && fabsf(s.sample.velocity) < .025f);
    CHECK(llabs(s.accumulatedTicks - (int64_t)lroundf((.6f + .55f) * 65536 / 6.283185307f)) < 20);
    CHECK(fabsf(s.fixedJoint.feedback.reactionB.force.y - 19.62f) < .01f);
    s.extraLoad = 1;
    CHECK(JointLab_Step(&s));
    CHECK(fabsf(s.fixedJoint.feedback.reactionB.force.y - 49.62f) < .01f);
    CHECK(fabsf(s.fixedJoint.feedback.reactionA.torque.z + 49.329f) < .02f);
    s.target = 2;
    for (i = 0; i < 1200; ++i)
        CHECK(JointLab_Step(&s));
    CHECK(fabsf(s.pendulum.joint.feedback.coordinate - 1.3962634f) < .003f);
    s.limits = 0;
    for (i = 0; i < 1800; ++i)
        CHECK(JointLab_Step(&s));
    CHECK(fabsf(s.pendulum.joint.feedback.coordinate - 2) < .003f);
    angle = s.pendulum.joint.feedback.coordinate;
    s.servo = 0;
    for (i = 0; i < 240; ++i)
        CHECK(JointLab_Step(&s));
    CHECK(fabsf(s.pendulum.joint.feedback.coordinate - angle) > .1f);
    CHECK(s.pendulum.joint.feedback.motorEffort == 0);
    puts("Joint lab: servo, encoder uint16/int16 accumulation, load reaction, limits and free "
         "swing passed");
    return 0;
}
