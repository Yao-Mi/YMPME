#include "ball.h"
#include "force_view.h"
#include "../common/force_channels.h"
#include <stdlib.h>
#define V YMPME_Vec3
static float length(pme_vec3 v) { return sqrtf(YMPME_VecDot(v, v)); }
int HandForce_Local(const pme_vec3 p[3], const float uv[6], pme_vec3 n, pme_vec3 f, pme_vec3 *out)
{
	return DemoForce_Local(p, uv, n, f, out);
}
int HandBall_LocalForce(const hand_ball_scene *s, size_t patch, pme_vec3 *out)
{
	if (!s || patch >= s->skin.count)
		return 0;
	pme_vec3 p[3];
	float uv[6];
	const uint16_t *tri = handTriangles[s->skin.faces[patch]];
	/* 同一步接触的patch法线与步末网格存在一个dt差；使用切向正交投影保持方向。 */
	for (int j = 0; j < 3; j++)
	{
		gre_fvector4d v = s->skin.hand.object->pointList[tri[j]].pos;
		p[j] = V(v.x * .1f, v.y * .1f, v.z * .1f);
		uv[j * 2] = s->sourceUv ? handUV[tri[j]][0] : s->autoCorners[patch * 6 + j * 2];
		uv[j * 2 + 1] = s->sourceUv ? handUV[tri[j]][1] : s->autoCorners[patch * 6 + j * 2 + 1];
	}
	return HandForce_Local(p, uv, s->skin.patches[patch].normal, s->skin.forces[patch], out);
}
GRErgb24 HandForce_RGB(pme_vec3 local, int *clipped)
{
	return DemoForce_RGB(local, .1f, .05f, clipped);
}
void HandBall_UpdateRGB(hand_ball_scene *s)
{
	pme_vec3 values[HAND_POINTS] = {{0}};
	float areas[HAND_POINTS] = {0};
	s->rgbClipped = 0;
	for (size_t p = 0; p < s->skin.count; p++)
	{
		pme_vec3 local = V(0, 0, 0);
		int clipped = 0;
		HandBall_LocalForce(s, p, &local);
		s->rgbPatch[p] = HandForce_RGB(local, &clipped);
		s->rgbClipped += clipped;
		const uint16_t *tri = handTriangles[s->skin.faces[p]];
		for (int k = 0; k < 3; k++)
		{
			values[tri[k]] =
				YMPME_VecAdd(values[tri[k]], YMPME_VecScale(local, s->skin.patches[p].area));
			areas[tri[k]] += s->skin.patches[p].area;
		}
	}
	for (int v = 0; v < HAND_POINTS; v++)
		s->rgbVertex[v] = HandForce_RGB(
			areas[v] > 0 ? YMPME_VecScale(values[v], 1 / areas[v]) : V(0, 0, 0), NULL);
}
void HandBall_RGBLegend(hand_ball_scene *s)
{
	GRE_RenderTarget target = YMGRE_Camera_GetRenderTarget(s->skin.uvLegend);
	for (int x = 0; x < HAND_UV_SIZE; x++)
	{
		int part = x * 3 / HAND_UV_SIZE, start = part * HAND_UV_SIZE / 3,
			end = (part + 1) * HAND_UV_SIZE / 3;
		float t = (float)(x - start) / (end - start - 1);
		pme_vec3 f = part == 0	 ? V(0, 0, .1f * t)
					 : part == 1 ? V(.1f * t - .05f, 0, 0)
								 : V(0, .1f * t - .05f, 0);
		GRErgb24 color = HandForce_RGB(f, NULL);
		for (int y = 0; y < 16; y++)
			target->data[y * HAND_UV_SIZE + x] = GRE_FramePixel_From_RGB24(color);
	}
}
static void pixel(GRE_RenderTarget t, int x, int y, GRErgb24 c)
{
	if (x >= 0 && y >= 0 && x < t->width && y < t->height)
		t->data[y * t->width + x] = GRE_FramePixel_From_RGB24(c);
}
static void line(GRE_RenderTarget t, int x, int y, int xx, int yy, GRErgb24 color)
{
	int dx = abs(xx - x), sx = x < xx ? 1 : -1, dy = -abs(yy - y), sy = y < yy ? 1 : -1,
		err = dx + dy;
	for (;;)
	{
		pixel(t, x, y, color);
		if (x == xx && y == yy)
			break;
		int e = err * 2;
		if (e >= dy)
		{
			err += dy;
			x += sx;
		}
		if (e <= dx)
		{
			err += dx;
			y += sy;
		}
	}
}
void HandBall_RenderForces(hand_ball_scene *s)
{
	s->arrowCount = 0;
	if (!s->arrows)
		return;
	GRE_RenderTarget image = YMGRE_Camera_GetRenderTarget(s->skin.uvCamera);
	const hand_uv_map *uv = HandBall_UV(s);
	gre_line3d lines[22 * 22 * 3];
	int count = 0;
	uint8_t shown[HAND_FACES] = {0};
	for (int y = 12; y < HAND_UV_SIZE; y += 24)
		for (int x = 12; x < HAND_UV_SIZE; x += 24)
		{
			int p = HandUV_Pick(uv, x, y);
			if (p < 0)
				continue;
			pme_vec3 f = s->skin.forces[p];
			float magnitude = length(f);
			GRErgb24 color = {255, 250, 225};
			/* 固定24像素网格。无载荷只画灰点；有载荷才显示箭头。 */
			pixel(image, x, y, (GRErgb24){125, 140, 155});
			if (magnitude < 1e-5f)
				continue;
			pme_vec3 local;
			if (!HandBall_LocalForce(s, (size_t)p, &local))
				continue;
			int tipX = x + (int)lroundf(12 * local.x / magnitude),
				tipY = y - 10 + (int)lroundf(12 * local.y / magnitude);
			/* 灰竖线/白参考点表示纯法向。头部相对此参考的上下左右位移表示UV切向。 */
			line(image, x, y, x, y - 10, (GRErgb24){75, 90, 105});
			pixel(image, x, y - 10, (GRErgb24){245, 245, 250});
			for (int offset = -2; offset <= 2; offset++)
			{
				line(image, x + offset, y, tipX + offset, tipY, (GRErgb24){10, 12, 20});
				line(image, x, y + offset, tipX, tipY + offset, (GRErgb24){10, 12, 20});
			}
			line(image, x, y, tipX, tipY, color);
			line(image, x + 1, y, tipX + 1, tipY, color);
			float dx = tipX - x, dy = tipY - y, l = sqrtf(dx * dx + dy * dy);
			if (l > 1)
			{
				int ax = (int)lroundf(3 * dx / l), ay = (int)lroundf(3 * dy / l);
				line(image, tipX, tipY, tipX - ax - ay, tipY - ay + ax, color);
				line(image, tipX, tipY, tipX - ax + ay, tipY - ay - ax, color);
			}
			++s->arrowCount;
			if (shown[p])
				continue;
			shown[p] = 1;
			/* 3D中是实际世界力方向：箭头尾位于皮肤外，箭头尖落在接触点。 */
			pme_vec3 end = YMPME_VecScale(s->skin.patches[p].position, 10),
					 dir = YMPME_VecScale(f, 1 / magnitude);
			pme_vec3 start = YMPME_VecSub(end, YMPME_VecScale(dir, .13f));
			pme_vec3 side = YMPME_VecCross(dir, fabsf(dir.z) < .9f ? V(0, 0, 1) : V(0, 1, 0));
			side = YMPME_VecScale(side, .025f / length(side));
			pme_vec3 head = YMPME_VecSub(end, YMPME_VecScale(dir, .035f));
			pme_vec3 a = YMPME_VecAdd(head, side), b = YMPME_VecSub(head, side);
			gre_fvector4d g0 = {start.x, start.y, start.z, 1}, g1 = {end.x, end.y, end.z, 1};
			lines[count++] = (gre_line3d){g0, g1, {255, 250, 220}, 2};
			lines[count++] = (gre_line3d){g1, {a.x, a.y, a.z, 1}, {255, 250, 220}, 2};
			lines[count++] = (gre_line3d){g1, {b.x, b.y, b.z, 1}, {255, 250, 220}, 2};
		}
	/* 方向标注不被球遮掉，便于查看接触侧；实体仍按正常深度渲染。 */
	YMGRE_Camera_LineList_Rendering(s->skin.hand.camera, lines, (uint32)count, 0);
}
