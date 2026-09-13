#ifndef SENSOR_LIDAR_PROJECTION_H
#define SENSOR_LIDAR_PROJECTION_H
#include "rig.h"
#include "YMGRE_RenderContext.h"
enum
{
	SENSOR_LIDAR_OFF = 0,
	SENSOR_LIDAR_POINTS = 1,
	SENSOR_LIDAR_LINES = 2,
	SENSOR_LIDAR_BOTH = 3
};
typedef struct sensor_lidar_projection_
{
	unsigned mode;
	size_t stride;
	int depthTest;
} sensor_lidar_projection;
/* Pure display conversion: never calls a ray provider or writes sensor state.
 * Actual trace endpoints are cached while the sensor performs its normal query.
 * A hit marker is a small three-axis cross. Caller supplies four slots. */
size_t SensorLidar_BeamLines(const sensor_lidar_segment *segment, unsigned mode, float scale,
							 gre_line3d out[4]);
void SensorLidar_Render(GRE_Camera4d camera, const sensor_lidar_segment *segments, size_t count,
						const sensor_lidar_projection *options, float scale);
const char *SensorLidar_ModeName(unsigned mode);
#endif
