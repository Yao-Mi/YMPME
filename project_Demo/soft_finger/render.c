#include "render.h"
#include "YMGRE_Creat.h"
#include "YMGRE_Free.h"
#include "YMGRE_Mem.h"
#include <string.h>
#include <stdlib.h>
#include "YMGRE_Camera.h"
#define V YMPME_Vec3
#define add YMPME_VecAdd
#define sub YMPME_VecSub
#define mul YMPME_VecScale
static GRErgb24 heat(float x)
{
	x = YMPME_Clamp(x, 0, 1);
	return (GRErgb24){(uint8)(25 + 230 * x), (uint8)(48 + 155 * sqrtf(x)), (uint8)(80 - 45 * x)};
}
static gre_fvector4d gre(pme_vec3 p) { return (gre_fvector4d){100 * p.x, 100 * p.y, 100 * p.z, 1}; }
int SoftView_Init(soft_view *v)
{
	memset(v, 0, sizeof(*v));
	if (!GraspScene_Init(&v->scene))
		return 0;
	YMGRE_Free_Camera(v->scene.camera);
	v->scene.camera = YMGRE_Creat_Camera(0, 840, 640, 22, 22, 20, 20);
	if (!v->scene.camera)
		return 0;
	YMGRE_Camera_Frustum_Init(v->scene.camera, 1, 2000);
	v->ballPart = -1;
	for (int i = 0; i < v->scene.count; ++i)
		if (v->scene.parts[i].shape == 0)
		{
			v->ballPart = i;
			grasp_part *p = &v->scene.parts[i];
			size_t bytes = (size_t)p->object->pointNum * sizeof(*v->ballRest);
			v->ballRest = malloc(bytes);
			if (!v->ballRest)
				return 0;
			memcpy(v->ballRest, p->local, bytes);
			break;
		}
	if (v->ballPart < 0)
		return 0;
	for (int side = 0; side < 2; ++side)
	{
		GRE_Object4d o = v->pads[side] =
			YMGRE_Creat_Object(9 * 17, GRASP_CELLS * 2, "soft fingertip", "");
		v->indentation[side] = YMGRE_Creat_RenderTarget(144, 288);
		if (!o || !v->indentation[side])
			return 0;
		o->boundType = GRE_Bounding_Sphere_R;
		o->BoundingSphereR = 50;
		for (int row = 0; row < 16; ++row)
			for (int col = 0; col < 8; ++col)
			{
				int a = row * 9 + col;
				int ids[6] = {a, a + 1, a + 9, a + 1, a + 10, a + 9};
				for (int t = 0; t < 2; ++t)
				{
					GRE_Polygon4d p = &o->polygonList[2 * (row * 8 + col) + t];
					p->index = GRE_PolyIndex_Malloc(3 * sizeof(*p->index));
					if (!p->index)
						return 0;
					p->num = 3;
					p->ishide = 0;
					for (int k = 0; k < 3; ++k)
						p->index[k] = (uint16)ids[t * 3 + (side ? 2 - k : k)];
				}
			}
		YMGRE_List_Append(&v->scene.objects, sizeof(*o), o);
	}
	return 1;
}
static float compression(const soft_finger *s, int side, int row, int col)
{
	float q = 0;
	int count = 0;
	for (int y = row - 1; y <= row; ++y)
		for (int x = col - 1; x <= col; ++x)
			if (x >= 0 && x < 8 && y >= 0 && y < 16)
			{
				q += s->nodes[side][y * 8 + x].compression;
				++count;
			}
	return q / count;
}
void SoftView_Draw(soft_view *v, const soft_finger *s, int camera, int wire)
{
	for (int side = 0; side < 2; ++side)
	{
		GRE_Object4d o = v->pads[side];
		const pme_rigid_body *b = &s->grasp.bodies[side + 1];
		pme_rotation r = YMPME_Quaternion_Rotation(b->orientation);
		o->WorldCoordinate = gre(b->position);
		for (int row = 0; row < 17; ++row)
			for (int col = 0; col < 9; ++col)
			{
				pme_vec3 local = V((side ? -1 : 1) * (.05f + s->material.maxCompression -
													  compression(s, side, row, col)),
								   (.5f - row / 16.f) * .8f, (col / 8.f - .5f) * .3f);
				o->pointList[row * 9 + col].pos =
					gre(add(b->position, YMPME_Rotation_Apply(r, local)));
			}
		for (int k = 0; k < GRASP_CELLS * 2; ++k)
		{
			GRE_Polygon4d p = &o->polygonList[k];
			gre_fvector4d a = o->pointList[p->index[0]].pos, bb = o->pointList[p->index[1]].pos,
						  c = o->pointList[p->index[2]].pos;
			pme_vec3 n = YMPME_VecCross(V(bb.x - a.x, bb.y - a.y, bb.z - a.z),
										V(c.x - a.x, c.y - a.y, c.z - a.z));
			p->pN = (gre_fvector4d){n.x, n.y, n.z, 0};
			p->planeColor = heat(s->nodes[side][k / 2].compression / .01f);
		}
	}
	/* The renderer and the contact query use the same affine scale, at 1:1. */
	grasp_part *ball = &v->scene.parts[v->ballPart];
	pme_vec3 scale = YMPME_SoftBall_Scale(&s->ball);
	for (int k = 0; k < ball->object->pointNum; ++k)
	{
		ball->local[k] = v->ballRest[k];
		ball->local[k].pos.x *= scale.x;
		ball->local[k].pos.y *= scale.y;
		ball->local[k].pos.z *= scale.z;
	}
	for (int k = 0; k < ball->object->polygonNum; ++k)
	{
		GRE_Polygon4d p = &ball->object->polygonList[k];
		gre_fvector4d a = ball->local[p->index[0]].pos, b = ball->local[p->index[1]].pos,
					  c = ball->local[p->index[2]].pos;
		pme_vec3 n =
			YMPME_VecCross(V(b.x - a.x, b.y - a.y, b.z - a.z), V(c.x - a.x, c.y - a.y, c.z - a.z));
		if (YMPME_VecDot(n, V(a.x, a.y, a.z)) < 0)
			n = mul(n, -1);
		ball->normals[k] = (gre_fvector4d){n.x, n.y, n.z, 0};
	}
	ball->object->BoundingSphereR = s->ball.radius * 100 * fmaxf(scale.x, fmaxf(scale.y, scale.z));
	/* Reuse the parent scene but suppress its point-contact markers; the soft
	 * pad mesh and our sampled maps display the distributed contacts instead. */
	grasp_lab display = s->grasp;
	memset(display.cells, 0, sizeof(display.cells));
	GraspScene_Render(&v->scene, &display, camera, wire, 1);
	for (int side = 0; side < 2; ++side)
		for (int map = 0; map < 2; ++map)
		{
			GRE_RenderTarget t = map ? v->indentation[side] : v->scene.maps[side];
			for (int y = 0; y < t->height; ++y)
				for (int x = 0; x < t->width; ++x)
				{
					int k = (y / 18) * 8 + x / 18;
					float amount = map ? s->nodes[side][k].compression / .01f
									   : s->grasp.cells[side][k].pressure / 2000.f;
					GRErgb24 color = heat(amount);
					if (x % 18 == 0 || y % 18 == 0)
						color = (GRErgb24){42, 53, 65};
					t->data[y * t->width + x] = GRE_FramePixel_From_RGB24(color);
				}
		}
}
void SoftView_Destroy(soft_view *v)
{
	free(v->ballRest);
	GraspScene_Destroy(&v->scene);
	for (int i = 0; i < 2; ++i)
	{
		if (v->pads[i])
			YMGRE_Free_Object(v->pads[i]);
		if (v->indentation[i])
			YMGRE_Free_RenderTarget(v->indentation[i]);
	}
}
