#include "YMPME_Lidar.h"
#include "../YMPME_RangeInternal.h"
pme_result YMPME_Lidar_Pattern(pme_vec3 *d, size_t capacity, size_t columns, size_t rows, float az,
							   float da, float el, float de)
{
	size_t i, j;
	double lastEl = (double)el + (rows ? rows - 1 : 0) * (double)de,
		   lastAz = (double)az + (columns ? columns - 1 : 0) * (double)da;
	if (!d || !columns || !rows || rows > SIZE_MAX / columns || !isfinite(az) || !isfinite(da) ||
		!isfinite(el) || !isfinite(de) || !isfinite(lastAz) || fabs(lastAz) > 1e6 ||
		fabs(az) > 1e6 || !isfinite(lastEl) || fabs(el) > 1.570797 || fabs(lastEl) > 1.570797)
		return PME_INVALID_ARGUMENT;
	if (capacity < columns * rows || columns * rows > SIZE_MAX / sizeof(*d))
		return PME_CAPACITY_EXCEEDED;
	for (i = 0; i < rows; ++i)
		for (j = 0; j < columns; ++j)
		{
			double a = az + j * (double)da, e = el + i * (double)de;
			d[i * columns + j] =
				YMPME_Vec3((float)(sin(a) * cos(e)), (float)sin(e), (float)(cos(a) * cos(e)));
		}
	return PME_OK;
}
pme_result YMPME_Lidar_Measure(const pme_lidar_desc *d, const pme_vec3 *beams, size_t n,
							   const pme_feedback_state *s, const pme_sensor_stamp *stamp,
							   const pme_ray_provider *provider, uint32_t *seed,
							   pme_lidar_frame *out)
{
	pme_vec3 origin;
	pme_rotation rotation;
	size_t i;
	uint32_t rng;
	if (!d || !beams || !n || !provider || !provider->cast || !seed || !out || !out->points ||
		!isfinite(d->minRange) || d->minRange < 0 || !isfinite(d->maxRange) ||
		d->maxRange <= d->minRange || !isfinite(d->bias) || !isfinite(d->noise) || d->noise < 0 ||
		!YMPME_Range_Pose(s, &d->mount, stamp, &origin, &rotation))
		return PME_INVALID_ARGUMENT;
	if (out->capacity < n || n > SIZE_MAX / sizeof(*out->points))
		return PME_CAPACITY_EXCEEDED;
	for (i = 0; i < n; ++i)
		if (!YMPME_Range_Unit(beams[i]))
			return PME_INVALID_ARGUMENT;
	rng = *seed;
	out->stamp.valid = 0;
	out->count = 0;
	for (i = 0; i < n; ++i)
	{
		pme_ray_hit hit;
		pme_result status =
			YMPME_Range_Cast(provider, origin, rotation, beams[i], d->maxRange, &hit);
		pme_lidar_point p = {{0, 0, 0}, NAN, SIZE_MAX, 0};
		float range;
		if (status != PME_OK)
			return status;
		if (hit.hit && hit.distance >= d->minRange)
		{
			range = hit.distance + d->bias;
			if (d->noise > 0)
				range += d->noise * YMPME_Sensor_UnitNoise(&rng);
			if (!isfinite(range))
				return PME_INVALID_ARGUMENT;
			if (range >= d->minRange && range <= d->maxRange)
			{
				p.range = range;
				p.point = YMPME_VecScale(beams[i], range);
				p.body = hit.body;
				p.valid = 1;
			}
		}
		out->points[i] = p;
	}
	out->origin = origin;
	out->rotation = rotation;
	out->stamp = *stamp;
	out->count = n;
	*seed = rng;
	return PME_OK;
}

pme_result YMPME_Lidar_Scan360(pme_vec3 *directions, size_t capacity, size_t count)
{
	if (!count)
		return PME_INVALID_ARGUMENT;
	return YMPME_Lidar_Pattern(directions, capacity, count, 1, 0,
							   (float)(6.2831853071795864769 / count), 0, 0);
}
