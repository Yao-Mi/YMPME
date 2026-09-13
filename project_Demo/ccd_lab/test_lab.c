#include "ccd_lab.h"
#include <stdio.h>
int main(void)
{
    for (int shape = 0; shape < 3; ++shape)
        for (int bounce = 0; bounce < 2; ++bounce)
        {
            ccd_lab s;
            if (!CCDLab_Init(&s, shape, bounce) || !CCDLab_Step(&s))
                return 1;
            if (s.lanes[0].bodies[1].position.x < .9f || s.lanes[1].bodies[1].position.x > 0 ||
                !s.lanes[1].hits)
                return 1;
            printf("shape %d bounce %d: discrete x %.3f / CCD x %.3f, %.3f ms, impulse %.2f Ns\n",
                   shape, bounce, s.lanes[0].bodies[1].position.x, s.lanes[1].bodies[1].position.x,
                   1000 * s.lanes[1].impactTime, s.lanes[1].impulse);
        }
    return 0;
}
