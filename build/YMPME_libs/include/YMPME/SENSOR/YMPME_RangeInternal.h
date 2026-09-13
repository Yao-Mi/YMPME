#ifndef YMPME_RANGE_INTERNAL_H
#define YMPME_RANGE_INTERNAL_H
#include "YMPME_Sensor.h"
#include "../CORE/YMPME_Ray.h"
int YMPME_Range_Pose(const pme_feedback_state *s, const pme_sensor_mount *mount,
					 const pme_sensor_stamp *stamp, pme_vec3 *origin, pme_rotation *rotation);
int YMPME_Range_Unit(pme_vec3 d);
pme_result YMPME_Range_Cast(const pme_ray_provider *provider, pme_vec3 origin,
							pme_rotation rotation, pme_vec3 direction, float maxDistance,
							pme_ray_hit *hit);
#endif
