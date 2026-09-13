#include "lidar_projection.h"
#include "YMGRE_Rendering_Pipeline.h"
static gre_fvector4d gre(pme_vec3 p, float scale)
{
	return (gre_fvector4d){p.x * scale, p.y * scale, p.z * scale, 1};
}
const char *SensorLidar_ModeName(unsigned mode)
{
	static const char *const names[] = {"OFF", "POINTS", "LINES", "POINTS+LINES"};
	return mode < 4 ? names[mode] : "OFF";
}
size_t SensorLidar_BeamLines(const sensor_lidar_segment *s, unsigned mode, float scale,
							 gre_line3d out[4])
{
	size_t n = 0;
	if (!s || !out || mode > 3 || !isfinite(scale) || scale <= 0)
		return 0;
	if (mode & SENSOR_LIDAR_LINES)
		out[n++] = (gre_line3d){gre(s->origin, scale), gre(s->end, scale),
								s->hit ? (GRErgb24){43, 164, 158} : (GRErgb24){66, 83, 97}, 1};
	if (s->hit && (mode & SENSOR_LIDAR_POINTS))
		for (int axis = 0; axis < 3; ++axis)
		{
			pme_vec3 d =
				YMPME_Vec3(axis == 0 ? .035f : 0, axis == 1 ? .035f : 0, axis == 2 ? .035f : 0);
			out[n++] = (gre_line3d){gre(YMPME_VecSub(s->end, d), scale),
									gre(YMPME_VecAdd(s->end, d), scale),
									{94, 255, 217},
									3};
		}
	return n;
}
void SensorLidar_Render(GRE_Camera4d camera, const sensor_lidar_segment *segments, size_t count,
						const sensor_lidar_projection *o, float scale)
{
	if (!camera || !segments || !o || !o->mode || o->mode > 3 || !o->stride)
		return;
	gre_line3d lines[64];
	size_t n = 0;
	for (size_t i = 0; i < count;)
	{
		if (n + 4 > 64)
		{
			YMGRE_Camera_LineList_Rendering(camera, lines, (uint32)n, o->depthTest);
			n = 0;
		}
		n += SensorLidar_BeamLines(&segments[i], o->mode, scale, &lines[n]);
		if (o->stride >= count - i)
			break;
		i += o->stride;
	}
	if (n)
		YMGRE_Camera_LineList_Rendering(camera, lines, (uint32)n, o->depthTest);
}
