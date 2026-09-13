#include "YMPME.h"
#include "BRIDGE/YMPME_GRE.h"
#include <stdio.h>
#include <string.h>
#define CHECK(x) do { if (!(x)) { fprintf(stderr, "line %d: %s\n", __LINE__, #x); return 1; } } while (0)
int main(void)
{
    gre_vertex4d local[2] = {{{-1, 0, 0, 1}, 0.2f, 0.3f}, {{1, 2, 0, 1}, 0.8f, 0.9f}};
    gre_vertex4d copy[2], points[2];
    gre_vertex4d_wN advanced[2] = {0}, temporary[2] = {0};
    gre_object4d object = {0};
    pme_body body;
    pme_body_desc d = YMPME_Body_DefaultDesc();
    int frame;
    memcpy(copy, local, sizeof(local));
    object.pointNum = 2;
    object.pointList = points;
    object.pointList_wN = advanced;
    object.pointList_wN_ = temporary;
    object.scale = 2;
    advanced[0].normal.y = 1;
    CHECK(YMPME_Body_Init(&body, &d) == PME_OK);
    for (frame = 0; frame < 1000; ++frame)
    {
        body.previousPosition = YMPME_Vec3(0, 0, 0);
        body.state.position = YMPME_Vec3(2, 4, 6);
        CHECK(YMPME_GRE_SyncObject(&body, &object, local, 2, 10, 0.5f) == PME_OK);
        CHECK(points[0].pos.x == 8 && points[1].pos.x == 12);
        CHECK(points[1].pos.y == 24 && points[0].pos.z == 30);
        CHECK(points[0].pos.w == 1 && object.WorldCoordinate.w == 1);
        CHECK(object.BoundingBoxMin.x == -1 && object.BoundingBoxMax.y == 2);
        CHECK(advanced[0].base.pos.x == 8 && temporary[1].base.pos.y == 24);
        CHECK(advanced[0].normal.y == 1 && points[0].u == local[0].u);
    }
    CHECK(memcmp(local, copy, sizeof(local)) == 0);
    CHECK(YMPME_GRE_SyncObject(&body, &object, points, 2, 10, 1) == PME_INVALID_ARGUMENT);
    CHECK(YMPME_GRE_SyncObject(&body, &object, local, 1, 10, 1) == PME_INVALID_ARGUMENT);
    CHECK(YMPME_GRE_SyncObject(&body, &object, local, 2, 0, 1) == PME_INVALID_ARGUMENT);
    puts("PASS GRE immutable local mesh, interpolation, units, attributes, bounds");
    return 0;
}
