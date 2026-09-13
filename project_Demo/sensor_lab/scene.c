#include "scene.h"
#include "YMGRE_BasicMesh_Gener.h"
#include "YMGRE_Rendering_Pipeline.h"
#include "YMGRE_Camera.h"
#include "YMGRE_Creat.h"
#include "YMGRE_Free.h"
#include <stdlib.h>
#include <string.h>
#define U 32.f
#define V YMPME_Vec3
static int add(sensor_scene *s, GRE_Object4d o, int owner, pme_vec3 offset)
{
	sensor_part *p;
	int i;
	if (!o)
		return 0;
	if (s->count >= 14)
	{
		YMGRE_Free_Object(o);
		return 0;
	}
	p = &s->parts[s->count++];
	p->object = o;
	p->body = owner;
	p->offset = offset;
	p->local = malloc((size_t)o->pointNum * sizeof(*p->local));
	p->normal = malloc((size_t)o->polygonNum * sizeof(*p->normal));
	if (!p->local || !p->normal)
		return 0;
	memcpy(p->local, o->pointList, (size_t)o->pointNum * sizeof(*p->local));
	for (i = 0; i < o->polygonNum; ++i)
		p->normal[i] = o->polygonList[i].pN;
	YMGRE_List_Append(&s->objects, sizeof(*o), o);
	return 1;
}
int SensorScene_Init(sensor_scene *s)
{
	int i;
	memset(s, 0, sizeof(*s));
	s->projection = (sensor_lidar_projection){SENSOR_LIDAR_BOTH, 5, 1};
	s->laserProjection = SENSOR_LIDAR_BOTH;
	if (!SensorRig_Init(&s->rig))
		return 0;
	for (i = 0; i < LAB_BODIES; ++i)
	{
		const pme_body_desc *b = &s->rig.bodies[i].state;
		GRE_Object4d o;
		GRErgb24 color = i == 11  ? (GRErgb24){60, 224, 225}
						 : i >= 8 ? (GRErgb24){237, 108, 85}
						 : i == 7 ? (GRErgb24){85, 155, 210}
						 : i >= 5 ? (GRErgb24){235, 179, 71}
								  : (GRErgb24){96, 117, 137};
		if (i == 0)
			o = YMGRE_MeshGener_RectPlane(8 * U, 10 * U, 16, 20, (GRErgb24){52, 66, 78},
										  "floor / skin", "");
		else if (b->shape == PME_SPHERE)
			o = YMGRE_MeshGener_Sphere(b->radius * U, 14, 20, color, "sphere", "");
		else
			o = YMGRE_MeshGener_Box(b->halfExtents.x * 2 * U, b->halfExtents.y * 2 * U,
									b->halfExtents.z * 2 * U, color, "obstacle", "");
		if (!add(s, o, i, i == 0 ? V(0, .1f, 0) : V(0, 0, 0)))
			return 0;
	}
	if (!add(s,
			 YMGRE_MeshGener_Box(.07f * U, .07f * U, .6f * U, (GRErgb24){245, 248, 240},
								 "sensor heading", ""),
			 11, V(0, 0, .3f)))
		return 0;
	s->ambient = YMGRE_Creat_Light(0, GRE_GlobalLight, (GRErgb24){255, 255, 255}, .7f);
	s->key = YMGRE_Creat_Light(1, GRE_PointLight, (GRErgb24){255, 246, 224}, .7f);
	if (!s->ambient || !s->key)
		return 0;
	s->key->pos = (gre_fvector4d){-200, 350, -220, 1};
	s->key->proper.kc0 = 1;
	s->key->proper.kc1 = s->key->proper.kc2 = 0;
	YMGRE_List_Append(&s->lights, sizeof(*s->ambient), s->ambient);
	YMGRE_List_Append(&s->lights, sizeof(*s->key), s->key);
	s->camera = YMGRE_Creat_Camera(0, 400, 400, 28, 28, 28, 28);
	s->radar = YMGRE_Creat_Camera(1, 400, 400, 28, 28, 28, 28);
	s->depth = YMGRE_Creat_Camera(2, 400, 240, 28, 28, 20, 20);
	s->skin = YMGRE_Creat_Camera(3, 400, 120, 28, 28, 10, 10);
	s->workspace = YMGRE_Creat_RenderWorkspace();
	if (!s->camera || !s->radar || !s->depth || !s->skin || !s->workspace)
		return 0;
	YMGRE_Camera_Frustum_Init(s->camera, 1, 2000);
	return 1;
}
static void pixel(GRE_RenderTarget t, int x, int y, GRErgb24 c)
{
	if (x >= 0 && y >= 0 && x < t->width && y < t->height)
		t->data[(size_t)y * t->width + x] = GRE_FramePixel_From_RGB24(c);
}
static void line(GRE_RenderTarget t, int x, int y, int ex, int ey, GRErgb24 c)
{
	int dx = abs(ex - x), sx = x < ex ? 1 : -1, dy = -abs(ey - y), sy = y < ey ? 1 : -1,
		error = dx + dy;
	for (;;)
	{
		int e;
		pixel(t, x, y, c);
		if (x == ex && y == ey)
			break;
		e = 2 * error;
		if (e >= dy)
		{
			error += dy;
			x += sx;
		}
		if (e <= dx)
		{
			error += dx;
			y += sy;
		}
	}
}
static GRErgb24 heat(float v)
{
	v = YMPME_Clamp(v, 0, 1);
	return (GRErgb24){(uint8)(35 + 220 * v), (uint8)(80 + 130 * (1 - fabsf(2 * v - 1))),
					  (uint8)(210 - 165 * v)};
}
static void plots(sensor_scene *s)
{
	GRE_RenderTarget r = YMGRE_Camera_GetRenderTarget(s->radar),
					 d = YMGRE_Camera_GetRenderTarget(s->depth),
					 t = YMGRE_Camera_GetRenderTarget(s->skin);
	int x, y, k;
	size_t i;
	float scale = 175 / s->rig.lidar.maxRange;
	for (y = 0; y < 400; ++y)
		for (x = 0; x < 400; ++x)
			pixel(r, x, y, (GRErgb24){19, 29, 39});
	for (k = 1; k <= 4; ++k)
		for (i = 0; i < 720; ++i)
		{
			float a = (float)i * 6.28318530718f / 720;
			pixel(r, 200 + (int)(175 * k * .25f * sinf(a)), 200 + (int)(175 * k * .25f * cosf(a)),
				  (GRErgb24){46, 64, 79});
		}
	line(r, 20, 200, 380, 200, (GRErgb24){46, 64, 79});
	line(r, 200, 20, 200, 380, (GRErgb24){46, 64, 79});
	for (i = 0; i < s->rig.scan.count; ++i)
	{
		const pme_lidar_point *p = &s->rig.scan.points[i];
		if (!p->valid)
			continue;
		x = 200 + (int)lroundf(p->point.x * scale);
		y = 200 - (int)lroundf(p->point.z * scale);
		if (i % 15 == 0)
			line(r, 200, 200, x, y, (GRErgb24){32, 77, 84});
		for (k = -1; k <= 1; ++k)
		{
			pixel(r, x + k, y, (GRErgb24){61, 224, 207});
			pixel(r, x, y + k, (GRErgb24){61, 224, 207});
		}
	}
	line(r, 200, 204, 200, 187, (GRErgb24){247, 194, 85});
	line(r, 200, 187, 196, 193, (GRErgb24){247, 194, 85});
	line(r, 200, 187, 204, 193, (GRErgb24){247, 194, 85});
	for (y = 0; y < 240; ++y)
		for (x = 0; x < 400; ++x)
		{
			float z = s->rig.image.depth[(y / 5) * LAB_DEPTH_WIDTH + x / 5];
			pixel(d, x, y,
				  isfinite(z) ? heat(1 - z / s->rig.depth.farDepth) : (GRErgb24){12, 15, 22});
		}
	for (y = 0; y < 120; ++y)
		for (x = 0; x < 400; ++x)
		{
			size_t at = (size_t)(y / 12) * 16 + x / 25;
			float pressure = s->rig.averagedCells[at].pressure;
			GRErgb24 c = pressure > 0 ? heat(pressure / 180) : (GRErgb24){26, 38, 51};
			if (x % 25 == 0 || y % 12 == 0)
				c = (GRErgb24){62, 76, 88};
			pixel(t, x, y, c);
		}
}
void SensorScene_Render(sensor_scene *s, int view)
{
	int i, k;
	gre_fvector4d eye, target;
	for (i = 0; i < s->count; ++i)
	{
		sensor_part *p = &s->parts[i];
		GRE_Object4d o = p->object;
		pme_vec3 position = s->rig.bodies[p->body].state.position;
		pme_rotation r = p->body == 11 ? s->rig.pose.rotation : YMPME_Rotation_Identity();
		pme_vec3 center = YMPME_VecAdd(position, YMPME_Rotation_Apply(r, p->offset));
		o->WorldCoordinate = (gre_fvector4d){center.x * U, center.y * U, center.z * U, 1};
		o->wireFrame = p->body >= 1 && p->body <= 4;
		o->isVisible = 1;
		for (k = 0; k < o->pointNum; ++k)
		{
			pme_vec3 v =
				YMPME_Rotation_Apply(r, V(p->local[k].pos.x, p->local[k].pos.y, p->local[k].pos.z));
			o->pointList[k] = p->local[k];
			o->pointList[k].pos =
				(gre_fvector4d){v.x + center.x * U, v.y + center.y * U, v.z + center.z * U, 1};
		}
		for (k = 0; k < o->polygonNum; ++k)
		{
			pme_vec3 n = YMPME_Rotation_Apply(r, V(p->normal[k].x, p->normal[k].y, p->normal[k].z));
			o->polygonList[k].pN = (gre_fvector4d){n.x, n.y, n.z, 0};
		}
	}
	eye =
		view ? (gre_fvector4d){0, 14 * U, -.01f * U, 1} : (gre_fvector4d){8 * U, 9 * U, -11 * U, 1};
	target = (gre_fvector4d){0, .5f * U, 0, 1};
	YMGRE_UVNCamera_PositionInit(s->camera, &eye, &target, NULL, 0);
	/* GRE UVN uses N x up for U. The sensor convention (+Z forward, +X right)
	 * requires the opposite horizontal axis. Adapt the camera basis and matrix
	 * together, so projection/culling remain coherent; no framebuffer mirroring. */
	s->camera->move.cu.x = -s->camera->move.cu.x;
	s->camera->move.cu.y = -s->camera->move.cu.y;
	s->camera->move.cu.z = -s->camera->move.cu.z;
	for (k = 0; k < 4; ++k)
		s->camera->move.TMat.val[0][k] = -s->camera->move.TMat.val[0][k];
	YMGRE_Camera_TanglePipline_RenderingWithWorkspace(s->camera, &s->lights, &s->objects,
													  &s->materials, s->workspace);
	SensorLidar_Render(s->camera, s->rig.lidarTrace, s->rig.lidarTraceCount, &s->projection, U);
#if PME_SENSOR_LASER
	if (s->rig.laserReading.stamp.valid)
	{
		const pme_laser_sample *p = &s->rig.laserReading;
		sensor_lidar_segment segment = {p->origin, p->rayEnd, p->hit};
		gre_line3d laserLines[4];
		size_t n = SensorLidar_BeamLines(&segment, s->laserProjection, U, laserLines);
		for (size_t i = 0; i < n; ++i)
		{
			laserLines[i].color = (s->laserProjection & SENSOR_LIDAR_LINES) && i == 0
									  ? (GRErgb24){255, 86, 112}
									  : (GRErgb24){255, 230, 92};
			laserLines[i].thickness = 3;
		}
		if (n)
			YMGRE_Camera_LineList_Rendering(s->camera, laserLines, (uint32)n,
											s->projection.depthTest);
	}
#endif
	plots(s);
}
static void keep(void *p) { (void)p; }
void SensorScene_Destroy(sensor_scene *s)
{
	int i;
	if (s->workspace)
		YMGRE_Free_RenderWorkspace(s->workspace);
	if (s->camera)
		YMGRE_Free_Camera(s->camera);
	if (s->radar)
		YMGRE_Free_Camera(s->radar);
	if (s->depth)
		YMGRE_Free_Camera(s->depth);
	if (s->skin)
		YMGRE_Free_Camera(s->skin);
	YMGRE_List_Clear(&s->objects, keep);
	YMGRE_List_Clear(&s->lights, keep);
	for (i = 0; i < s->count; ++i)
	{
		free(s->parts[i].local);
		free(s->parts[i].normal);
		YMGRE_Free_Object(s->parts[i].object);
	}
	if (s->ambient)
		YMGRE_Free_Light(s->ambient);
	if (s->key)
		YMGRE_Free_Light(s->key);
	if (s->rig.world.bodies)
		YMPME_World_Destroy(&s->rig.world);
}

static float projectedX(const sensor_scene *s, int body)
{
	pme_vec3 p = s->rig.bodies[body].state.position;
	const gre_fmat4x4 *m = &s->camera->move.TMat;
	float x =
		m->val[0][0] * p.x * U + m->val[0][1] * p.y * U + m->val[0][2] * p.z * U + m->val[0][3];
	float z =
		m->val[2][0] * p.x * U + m->val[2][1] * p.y * U + m->val[2][2] * p.z * U + m->val[2][3];
	return z > 0 ? x / z : NAN;
}
int SensorScene_CheckHandedness(sensor_scene *s)
{
	int view;
	for (view = 0; view < 2; ++view)
	{
		SensorScene_Render(s, view);
		/* Far gold box is world +X; blue sphere is world -X, both at z=2.
		 * Both world views must place the box right of the sphere. */
		if (!(projectedX(s, 5) > projectedX(s, 7)))
			return 0;
		if (!(s->camera->move.cu.x > 0))
			return 0;
		/* The cached basis must match the actual projection matrix. */
		if (fabsf(s->camera->move.cu.x - s->camera->move.TMat.val[0][0]) > 1e-6f)
			return 0;
	}
	return 1;
}
