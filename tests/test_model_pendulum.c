#include "MODEL/PENDULUM/YMPME_Pendulum.h"
#include <stdio.h>
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
    pme_pendulum p;
    pme_pendulum_config c = YMPME_Pendulum_DefaultConfig();
    int i, crossings = 0;
    float last;
    double crossTime[4] = {0};
    pme_vec3 g = YMPME_Vec3(0, -9.81f, 0);
    c.initialAngle = .1f;
    CHECK(YMPME_Pendulum_Init(&p, &c) == PME_OK);
    last = c.initialAngle;
    for (i = 0; i < 2400; ++i)
    {
        CHECK(YMPME_Pendulum_Step(&p, g, 1.f / 240) == PME_OK);
        if (last > 0 && p.joint.feedback.coordinate <= 0 && crossings < 4)
            crossTime[crossings++] = p.time;
        last = p.joint.feedback.coordinate;
        CHECK(p.joint.feedback.anchorError < .001f);
    }
    CHECK(crossings == 4);
    CHECK(fabs((crossTime[1] - crossTime[0]) -
               2 * 3.141592653589793 *
                   sqrt((c.length * c.length + .4 * c.radius * c.radius) / (9.81 * c.length))) <
          .02);
    /* PD control is external; equilibrium torque balances gravity. */
    for (i = 0; i < 2400; ++i)
    {
        float effort = 40 * (.5f - p.joint.feedback.coordinate) - 10 * p.joint.feedback.speed;
        effort += c.mass * 9.81f * c.length * sinf(p.joint.feedback.coordinate);
        CHECK(YMPME_Joint_SetMotor(
                  &p.joint, (pme_joint_motor){PME_JOINT_MOTOR_EFFORT, 0, effort, 25}) == PME_OK);
        CHECK(YMPME_Pendulum_Step(&p, g, 1.f / 240) == PME_OK);
    }
    CHECK(fabsf(p.joint.feedback.coordinate - .5f) < .003f);
    CHECK(fabsf(p.joint.feedback.motorEffort - c.mass * 9.81f * c.length * sinf(.5f)) < .05f);
    CHECK(fabsf(p.body.feedback.acceleration.y) < .01f);
    printf("Pendulum: period %.5f s, servo %.4f rad, torque %.4f N m\n",
           crossTime[1] - crossTime[0], p.joint.feedback.coordinate, p.joint.feedback.motorEffort);
    return 0;
}
