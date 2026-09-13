#include "YMPME_Depth.h"
#include "../YMPME_RangeInternal.h"
static int valid(const pme_depth_desc *d)
{
	return d && d->width && d->height && d->height <= SIZE_MAX / d->width &&
		   d->width * d->height <= SIZE_MAX / sizeof(pme_vec3) && isfinite(d->fx) && d->fx > 0 &&
		   isfinite(d->fy) && d->fy > 0 && isfinite(d->cx) && isfinite(d->cy) &&
		   isfinite(d->nearDepth) && d->nearDepth > 0 && isfinite(d->farDepth) &&
		   d->farDepth > d->nearDepth && isfinite(d->bias) && isfinite(d->noise) && d->noise >= 0;
}
pme_result YMPME_Depth_BuildRays(const pme_depth_desc *d, pme_vec3 *rays, size_t capacity)
{
	size_t x, y;
	if (!valid(d) || !rays)
		return PME_INVALID_ARGUMENT;
	if (capacity < d->width * d->height)
		return PME_CAPACITY_EXCEEDED;
	/* Validate corner magnitudes before touching the caller buffer. */
	if (fabs(((double)d->width + .5 - d->cx) / d->fx) > 1e6 ||
		fabs((.5 - (double)d->cx) / d->fx) > 1e6 ||
		fabs(((double)d->height + .5 - d->cy) / d->fy) > 1e6 ||
		fabs((.5 - (double)d->cy) / d->fy) > 1e6)
		return PME_INVALID_ARGUMENT;
	for (y = 0; y < d->height; ++y)
		for (x = 0; x < d->width; ++x)
		{
			pme_vec3 v = YMPME_Vec3((float)(((double)x + .5 - d->cx) / d->fx),
									(float)((d->cy - y - .5) / d->fy), 1);
			rays[y * d->width + x] = YMPME_VecScale(v, 1 / sqrtf(YMPME_VecDot(v, v)));
		}
	return PME_OK;
}
pme_result YMPME_Depth_Measure(const pme_depth_desc *d, const pme_vec3 *rays, size_t n,
							   const pme_feedback_state *s, const pme_sensor_stamp *stamp,
							   const pme_ray_provider *provider, uint32_t *seed,
							   pme_depth_frame *out)
{
	size_t i, count;
	pme_vec3 origin;
	pme_rotation rotation;
	uint32_t rng;
	if (!valid(d) || !rays || !provider || !provider->cast || !seed || !out || !out->depth ||
		!YMPME_Range_Pose(s, &d->mount, stamp, &origin, &rotation))
		return PME_INVALID_ARGUMENT;
	count = d->width * d->height;
	if (n < count || out->capacity < count)
		return PME_CAPACITY_EXCEEDED;
	for (i = 0; i < count; ++i)
		if (!YMPME_Range_Unit(rays[i]) || rays[i].z <= 0 || !isfinite(d->farDepth / rays[i].z))
			return PME_INVALID_ARGUMENT;

	/* Cache identity check: intrinsics are mutable descriptors, so validate ray slopes.
	 * Uses ratios only; normalization/sqrt stays in BuildRays. */
	for (i = 0; i < count; ++i)
	{
		double x = ((double)(i % d->width) + .5 - d->cx) / d->fx;
		double y = (d->cy - (double)(i / d->width) - .5) / d->fy;
		if (fabs(rays[i].x / rays[i].z - x) > 1e-5 * fmax(1, fabs(x)) ||
			fabs(rays[i].y / rays[i].z - y) > 1e-5 * fmax(1, fabs(y)))
			return PME_INVALID_ARGUMENT;
	}
	rng = *seed;
	out->stamp.valid = 0;
	out->width = out->height = 0;
	for (i = 0; i < count; ++i)
	{
		pme_ray_hit hit;
		pme_result status =
			YMPME_Range_Cast(provider, origin, rotation, rays[i], d->farDepth / rays[i].z, &hit);
		float z = NAN;
		if (status != PME_OK)
			return status;
		if (hit.hit && hit.distance * rays[i].z >= d->nearDepth)
		{
			z = hit.distance * rays[i].z + d->bias;
			if (d->noise > 0)
				z += d->noise * YMPME_Sensor_UnitNoise(&rng);
			if (!isfinite(z))
				return PME_INVALID_ARGUMENT;
			if (z < d->nearDepth || z > d->farDepth)
				z = NAN;
		}
		out->depth[i] = z;
	}
	out->origin = origin;
	out->rotation = rotation;
	out->stamp = *stamp;
	out->width = d->width;
	out->height = d->height;
	*seed = rng;
	return PME_OK;
}
static int frameValid(const pme_depth_frame *f)
{
	return f && f->stamp.valid && f->depth && f->width && f->height &&
		   f->height <= SIZE_MAX / f->width && f->capacity >= f->width * f->height;
}
pme_result YMPME_Depth_EncodeU16(const pme_depth_frame *f, float units, uint16_t *out,
								 size_t capacity)
{
	size_t i;
	if (!frameValid(f) || !out || !isfinite(units) || units <= 0)
		return PME_INVALID_ARGUMENT;
	if (capacity < f->width * f->height)
		return PME_CAPACITY_EXCEEDED;
	for (i = 0; i < f->width * f->height; ++i)
	{
		double v = floor((double)f->depth[i] * units + .5);
		out[i] = isfinite(v) && v >= 1 && v <= 65535 ? (uint16_t)v : 0;
	}
	return PME_OK;
}
pme_result YMPME_Depth_Unproject(const pme_depth_frame *f, const pme_vec3 *rays, size_t n,
								 pme_vec3 *out, size_t capacity)
{
	size_t i, count;
	if (!frameValid(f) || !rays || !out)
		return PME_INVALID_ARGUMENT;
	count = f->width * f->height;
	if (n < count || capacity < count)
		return PME_CAPACITY_EXCEEDED;
	for (i = 0; i < count; ++i)
		if (!YMPME_Range_Unit(rays[i]) || rays[i].z <= 0)
			return PME_INVALID_ARGUMENT;
	for (i = 0; i < count; ++i)
		out[i] = isfinite(f->depth[i]) && f->depth[i] > 0
					 ? YMPME_VecScale(rays[i], f->depth[i] / rays[i].z)
					 : YMPME_Vec3(NAN, NAN, NAN);
	return PME_OK;
}
