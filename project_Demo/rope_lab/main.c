#include "BRIDGE/YMPME_RopeGRE.h"
#include "YMGRE_BasicMesh_Gener.h"
#include "YMGRE_Camera.h"
#include "YMGRE_Creat.h"
#include "YMGRE_Free.h"
#include "YMGRE_List.h"
#include "YMGRE_Rendering_Pipeline.h"
#include "YMGUI_Invalidate.h"
#include "YMGUI_Label.h"
#include "demo_host.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define N 33
#define U 50.0f
typedef struct rig_
{
	pme_rope rope;
	pme_rope_node nodes[N];
	pme_rope_link links[N - 1];
	pme_rope_gre binding;
	gre_list objects, lights, materials;
	GRE_Object4d markers[2];
	gre_vertex4d *local[2];
	GRE_Light4d ambient, key;
	GRE_Camera4d camera;
	GRE_RenderWorkspace workspace;
	double syncUs, renderUs;
	size_t bytes;
} rig;
static void keep(void *p) { (void)p; }
static void destroy(rig *s)
{
	if (s->workspace)
		YMGRE_Free_RenderWorkspace(s->workspace);
	if (s->camera)
		YMGRE_Free_Camera(s->camera);
	YMGRE_List_Clear(&s->objects, keep);
	YMGRE_List_Clear(&s->lights, keep);
	YMPME_GRE_Rope_Destroy(&s->binding);
	for (int i = 0; i < 2; ++i)
	{
		if (s->markers[i])
			YMGRE_Free_Object(s->markers[i]);
		free(s->local[i]);
	}
	if (s->ambient)
		YMGRE_Free_Light(s->ambient);
	if (s->key)
		YMGRE_Free_Light(s->key);
	memset(s, 0, sizeof(*s));
}
static int resetModel(rig *s, int kind)
{
	if (kind == 1)
	{
		/* 2.1m 绳：两条竖直腿 + 半圆弯底，重物从靠近锚点的同高度绳端释放。 */
		const float length = 2.1f, radius = .25f, top = 1.3f;
		const float pi = 3.14159265358979323846f;
		const float leg = (length - pi * radius) * .5f;
		pme_vec3 positions[17];
		float masses[17], rest[16];
		for (int i = 0; i < 17; ++i)
		{
			float arc = length * i / 16;
			if (arc < leg)
				positions[i] = YMPME_Vec3(0, top - arc, 0);
			else if (arc < leg + pi * radius)
			{
				float angle = pi + (arc - leg) / radius;
				positions[i] =
					YMPME_Vec3(radius + radius * cosf(angle), top - leg + radius * sinf(angle), 0);
			}
			else
				positions[i] = YMPME_Vec3(2 * radius, top - leg + arc - leg - pi * radius, 0);
			masses[i] = i == 0 ? 0 : (i == 16 ? 2.0f : .04f);
			if (i < 16)
				rest[i] = length / 16;
		}
		positions[0] = YMPME_Vec3(0, top, 0);
		positions[16] = YMPME_Vec3(2 * radius, top, 0);
		pme_rope_desc desc = YMPME_Rope_DefaultDesc();
		desc.count = 17;
		desc.positions = positions;
		desc.masses = masses;
		desc.restLengths = rest;
		if (YMPME_Rope_Init(&s->rope, &desc, s->nodes, N, s->links, N - 1) != PME_OK)
			return 0;
	}
	else if (YMPME_Rope_InitSpan(&s->rope, s->nodes, N, s->links, N - 1, N,
								 YMPME_Vec3(-1.4f, 1.1f, 0), YMPME_Vec3(1.4f, 1.1f, 0), 3.8f, .04f,
								 1, 1) != PME_OK)
		return 0;
	s->rope.iterations = 64;
	s->rope.damping = kind == 1 ? .03f : (kind == 0 ? 2.0f : .5f);
	return 1;
}
static int init(rig *s, int kind)
{
	memset(s, 0, sizeof(*s));
	if (!resetModel(s, kind))
		return 0;
	if (YMPME_GRE_Rope_Create(&s->binding, &s->rope, U, .023f, (GRErgb24){244, 198, 101}) != PME_OK)
		return 0;
	YMGRE_List_Append(&s->objects, sizeof(*s->binding.object), s->binding.object);
	for (int i = 0; i < 2; ++i)
	{
		s->markers[i] = YMGRE_MeshGener_Sphere(
			kind == 1 && i == 1 ? 9 : 4, 10, 16,
			i == 1 ? (GRErgb24){54, 175, 220} : (GRErgb24){236, 111, 72}, "anchor-or-load", "");
		if (!s->markers[i])
			return 0;
		s->local[i] = malloc(s->markers[i]->pointNum * sizeof(*s->local[i]));
		if (!s->local[i])
			return 0;
		memcpy(s->local[i], s->markers[i]->pointList,
			   s->markers[i]->pointNum * sizeof(*s->local[i]));
		YMGRE_List_Append(&s->objects, sizeof(*s->markers[i]), s->markers[i]);
	}
	s->ambient = YMGRE_Creat_Light(0, GRE_GlobalLight, (GRErgb24){255, 255, 255}, .55f);
	s->key = YMGRE_Creat_Light(1, GRE_PointLight, (GRErgb24){255, 246, 226}, .9f);
	if (!s->ambient || !s->key)
		return 0;
	s->key->pos = (gre_fvector4d){-150, 200, -200, 1};
	s->key->proper.kc0 = 1;
	s->key->proper.kc1 = s->key->proper.kc2 = 0;
	YMGRE_List_Append(&s->lights, sizeof(*s->ambient), s->ambient);
	YMGRE_List_Append(&s->lights, sizeof(*s->key), s->key);
	float beta = atanf(tanf(38 * 3.14159265f / 180) * 460 / 400) * 180 / 3.14159265f;
	s->camera = YMGRE_Creat_Camera(0, 400, 460, 38, 38, beta, beta);
	s->workspace = YMGRE_Creat_RenderWorkspace();
	if (!s->camera || !s->workspace)
		return 0;
	YMGRE_Camera_Frustum_Init(s->camera, 1, 2000);
	return 1;
}
static int render(rig *s, int angled, float alpha)
{
	double start = YMGRE_DemoHost_Time();
	if (YMPME_GRE_Rope_Sync(&s->binding, alpha) != PME_OK)
		return 0;
	s->bytes = s->binding.bytesWritten;
	for (int i = 0; i < 2; ++i)
	{
		GRE_Object4d o = s->markers[i];
		const pme_rope_node *n = &s->nodes[i ? s->rope.count - 1 : 0];
		pme_vec3 pos = YMPME_VecScale(YMPME_VecAdd(YMPME_VecScale(n->previous, 1 - alpha),
												   YMPME_VecScale(n->position, alpha)),
									  U);
		o->WorldCoordinate = (gre_fvector4d){pos.x, pos.y, pos.z, 1};
		for (int k = 0; k < o->pointNum; ++k)
		{
			o->pointList[k] = s->local[i][k];
			o->pointList[k].pos.x += pos.x;
			o->pointList[k].pos.y += pos.y;
			o->pointList[k].pos.z += pos.z;
		}
	}
	double after = YMGRE_DemoHost_Time();
	s->syncUs = (double)(after - start) * 1e6;
	gre_fvector4d eye = {angled ? 65 : 0, angled ? 30 : 0, -150, 1}, target = {0, 0, 0, 1};
	YMGRE_UVNCamera_PositionInit(s->camera, &eye, &target, NULL, 0);
	YMGRE_Camera_TanglePipline_RenderingWithWorkspace(s->camera, &s->lights, &s->objects,
													  &s->materials, s->workspace);
	s->renderUs = (double)(YMGRE_DemoHost_Time() - after) * 1e6;
	return 1;
}
enum
{
	PAUSE = 1,
	STEP = 2,
	RESET = 4,
	KICK = 8,
	AUTO = 16,
	VIEW = 32,
	RELEASE = 64,
	QUIT = 128,
	SLOW = 256
};
static const YMGRE_DemoKeyBinding keyBindings[] = {
	{'s', 0, SLOW, 0},
	{YMGRE_KEY_SPACE, 0, PAUSE, 0},
	{'n', 0, STEP, 0},
	{YMGRE_KEY_HOME, 0, RESET, 0},
	{'k', 0, KICK, 0},
	{'t', 0, AUTO, 0},
	{'c', 0, VIEW, 0},
	{'p', 0, RELEASE, 0},
	{YMGRE_KEY_ESCAPE, 0, QUIT, 0},
};

static GYOBJ label(GYOBJ root, int x, int y, int width, const char *t)
{
	GYOBJ p = YMGUI_Creat_Label_Creat(root, x, y, width, 22);
	if (p)
	{
		YMGUI_Label_SetText(p, t);
		YMGUI_Label_SetTextColor(p, GY_ARGB(255, 227, 233, 239));
	}
	return p;
}
static uint64_t hash(GRE_RenderTarget t)
{
	size_t n = (size_t)t->width * t->height * sizeof(*t->data);
	const unsigned char *p = (const unsigned char *)t->data;
	uint64_t h = UINT64_C(14695981039346656037);
	for (size_t i = 0; i < n; ++i)
		h = (h ^ p[i]) * UINT64_C(1099511628211);
	return h;
}
static int advance(rig *s, int automatic, int slow)
{
	if (automatic && s[2].nodes[N - 1].inverseMass == 0)
	{
		double t = s[2].rope.time + 1.0 / 120;
		if (YMPME_Rope_SetAnchor(&s[2].rope, N - 1,
								 YMPME_Vec3(1.4f + .35f * sinf((float)t * 1.2f),
											1.1f + .35f * sinf((float)t * 1.7f),
											.3f * sinf((float)t))) != PME_OK)
			return 0;
	}
	for (int i = 0; i < 3; ++i)
		if (YMPME_Rope_Step(&s[i].rope, (i == 1 && slow ? .25f : 1.0f) / 120) != PME_OK)
			return 0;
	return 1;
}
int main(int argc, char **argv)
{
	static rig s[3];
	YMGRE_DemoHost host;

	GYOBJ images[3], stats[3][3], info;
	const char *names[] = {"1  FIXED ENDS / NATURAL SAG", "2  U-SHAPED ROPE / FALLING LOAD",
						   "3  MOVING ANCHOR / 3D ROPE"};
	int smoke = argc == 2 && !strcmp(argv[1], "--smoke-test"), ready = 0, result = 1, paused = 1,
		automatic = 1, slow = 1, view = 0, frame = 0, title = 0, changed = 0, tested = 0;
	double previous;
	uint64_t first[3] = {0};
	double accumulator = 0, saved = 0, physicsUs = 0;
	if (argc > 1 && !smoke)
	{
		fprintf(stderr, "Usage: %s [--smoke-test]\n", argv[0]);
		return 1;
	}
	for (int i = 0; i < 3; ++i)
		if (!init(&s[i], i))
			goto done;
	YMGRE_DemoHost_DefaultScale(1);
	if (!YMGRE_DemoHost_Init(&host, 1200, 700, 40))
		goto done;
	ready = 1;
	for (int i = 0; i < 3; ++i)
	{
		if (!label(host.context->root, i * 400, 4, 400, names[i]))
			goto done;
		images[i] = YMGRE_DemoHost_AddTarget(&host, YMGRE_Camera_GetRenderTarget(s[i].camera),
											 i * 400, 30, 400, 460);
		if (!images[i])
			goto done;
		for (int j = 0; j < 3; ++j)
			if (!(stats[i][j] = label(host.context->root, i * 400, 498 + j * 24, 400, "")))
				goto done;
	}
	info = label(host.context->root, 0, 590, 1200, "");
	if (!info ||
		!label(host.context->root, 0, 622, 600, "SPACE Release/pause  N Step  K Push ropes") ||
		!label(host.context->root, 600, 622, 600, "T Anchor motion  P Release/fix right end") ||
		!label(host.context->root, 0, 656, 600, "C Front/angled  HOME Reset  ESC Exit") ||
		!label(host.context->root, 600, 656, 600, "S Middle panel: 0.25x / 1x speed"))
		goto done;
	if (!YMGRE_DemoHost_BindKeys(&host, keyBindings, sizeof keyBindings / sizeof keyBindings[0]))
		goto done;

	previous = YMGRE_DemoHost_Time();

	puts("ROPE model: SPACE releases the U-shaped rope-end load; S slow motion; HOME restores the "
		 "paused "
		 "initial U.");
	fflush(stdout);
	for (;; ++frame)
	{
		double now = YMGRE_DemoHost_Time();
		double elapsed = smoke ? 1.0 / 60 : (double)(now - previous);
		previous = now;
		if (!title)
		{

			if (host.context)
			{
				YMGRE_DemoHost_SetTitle(
					&host, "YMPME MODEL/ROPE + GRE | Sag, suspended load and moving anchor");
				title = 1;
			}
		}
		if (smoke)
		{
			if (frame == 0)
			{
				if (s[1].nodes[16].position.y != s[1].nodes[0].position.y ||
					fabs(s[1].nodes[16].position.x - s[1].nodes[0].position.x - .5f) > 1e-6f ||
					s[1].nodes[8].position.y > .45f ||
					YMPME_VecDot(s[1].nodes[16].velocity, s[1].nodes[16].velocity) != 0)
					goto done;
				if (!YMGRE_DemoHost_InjectKey(&host, YMGRE_KEY_SPACE, 1))
					goto done;
			}
			if (frame == 30)
			{
				if (s[1].nodes[16].position.y >= 1.28f ||
					fabs(s[1].rope.time - s[0].rope.time * .25) > 1e-6)
					goto done;
				saved = s[0].rope.time;
				if (!YMGRE_DemoHost_InjectKey(&host, YMGRE_KEY_SPACE, 1))
					goto done;
			}
			if (frame == 35)
			{
				if (s[0].rope.time != saved)
					goto done;
				tested |= 1;
				if (!YMGRE_DemoHost_InjectKey(&host, 'n', 1))
					goto done;
			}
			if (frame == 36)
			{
				if (fabs(s[0].rope.time - saved - 1.0 / 120) > 1e-7)
					goto done;
				tested |= 2;
			}
			if (frame == 40 && (!YMGRE_DemoHost_InjectKey(&host, YMGRE_KEY_SPACE, 1) ||
								!YMGRE_DemoHost_InjectKey(&host, 'k', 1) ||
								!YMGRE_DemoHost_InjectKey(&host, 'c', 1)))
				goto done;
			if (frame == 50 && !YMGRE_DemoHost_InjectKey(&host, 'p', 1))
				goto done;
			if (frame == 51)
			{
				if (s[2].nodes[N - 1].inverseMass == 0)
					goto done;
				tested |= 4;
			}
			if (frame == 70 && (!YMGRE_DemoHost_InjectKey(&host, 'p', 1) ||
								!YMGRE_DemoHost_InjectKey(&host, 't', 1) ||
								!YMGRE_DemoHost_InjectKey(&host, 's', 1)))
				goto done;
			if (frame == 71)
			{
				if (s[2].nodes[N - 1].inverseMass != 0 || automatic || slow)
					goto done;
				tested |= 8;
			}
			if (frame == 80 && !YMGRE_DemoHost_InjectKey(&host, YMGRE_KEY_HOME, 1))
				goto done;
			if (frame == 81)
			{
				if (s[0].rope.time != 0 || !automatic || !paused || !slow ||
					s[1].nodes[16].position.y != 1.3f)
					goto done;
				tested |= 16;
			}
			if (frame == 85 && !YMGRE_DemoHost_InjectKey(&host, YMGRE_KEY_SPACE, 1))
				goto done;
			if (frame == 110 && !YMGRE_DemoHost_InjectKey(&host, YMGRE_KEY_ESCAPE, 1))
				goto done;
		}
		int a = YMGRE_DemoHost_TakeActions(&host), steps = 0;
		if (a & QUIT)
		{
			result = smoke ? !(changed == 7 && tested == 31 && view == 1) : 0;
			break;
		}
		if (a & RESET)
		{
			for (int i = 0; i < 3; ++i)
				if (!resetModel(&s[i], i) ||
					YMPME_GRE_Rope_Rebind(&s[i].binding, &s[i].rope) != PME_OK)
					goto done;
			paused = 1;
			slow = 1;
			automatic = 1;
			accumulator = 0;
			elapsed = 0;
		}
		if (a & PAUSE)
		{
			paused = !paused;
			accumulator = 0;
			elapsed = 0;
		}
		if (a & SLOW)
			slow = !slow;
		if (a & AUTO)
			automatic = !automatic;
		if (a & VIEW)
			view = !view;
		if (a & RELEASE)
		{
			float mass = s[2].nodes[N - 1].inverseMass ? 0 : .04f;
			if (YMPME_Rope_SetMass(&s[2].rope, N - 1, mass) != PME_OK)
				goto done;
		}
		if (a & KICK)
			for (int i = 0; i < 3; ++i)
			{
				size_t k = i == 1 ? s[i].rope.count - 1 : s[i].rope.count / 2;
				float mass = 1 / s[i].nodes[k].inverseMass;
				if (YMPME_Rope_AddImpulse(&s[i].rope, k, YMPME_Vec3(mass * .6f, 0, mass * .4f)) !=
					PME_OK)
					goto done;
			}
		double tick = YMGRE_DemoHost_Time();
		if (paused && (a & STEP))
		{
			if (!advance(s, automatic, slow))
				goto done;
			steps = 1;
		}
		if (!paused)
		{
			accumulator += fmin(elapsed, .2);
			while (accumulator >= 1.0 / 120 && steps < 24)
			{
				if (!advance(s, automatic, slow))
					goto done;
				accumulator -= 1.0 / 120;
				++steps;
			}
		}
		if (steps)
			physicsUs = (double)(YMGRE_DemoHost_Time() - tick) * 1e6 / steps;
		float alpha = paused ? 1 : (float)(accumulator * 120);
		for (int i = 0; i < 3; ++i)
		{
			if (!render(&s[i], view, alpha))
				goto done;
			YMGUI_Obj_Invalidate(images[i]);
			if (smoke)
			{
				uint64_t h = hash(YMGRE_Camera_GetRenderTarget(s[i].camera));
				if (!frame)
					first[i] = h;
				else if (h != first[i])
					changed |= 1 << i;
			}
		}
		if (frame % 4 == 0 || a)
		{
			char text[64];
			for (int i = 0; i < 3; ++i)
			{
				double worst = 0;
				for (size_t k = 0; k < s[i].rope.linkCount; ++k)
				{
					pme_vec3 d = YMPME_VecSub(s[i].nodes[k + 1].position, s[i].nodes[k].position);
					worst = fmax(worst, sqrt(YMPME_VecDot(d, d)) / s[i].links[k].restLength - 1);
				}
				if (i == 1)
					snprintf(text, sizeof(text), "U release %.2fx | Stretch %.2f%%", slow ? .25 : 1,
							 worst * 100);
				else
					snprintf(text, sizeof(text), "%zu nodes | Stretch %.2f%%", s[i].rope.count,
							 worst * 100);
				YMGUI_Label_SetText(stats[i][0], text);
				snprintf(text, sizeof(text), "Sync %.1f us | Draw %.2f ms", s[i].syncUs,
						 s[i].renderUs / 1000);
				YMGUI_Label_SetText(stats[i][1], text);
				snprintf(text, sizeof(text), "Mesh %d verts | Write %.1f KiB",
						 s[i].binding.object->pointNum, s[i].bytes / 1024.0);
				YMGUI_Label_SetText(stats[i][2], text);
			}
			snprintf(text, sizeof(text), "%s | Anchor %s | Physics %.2f ms / step (all 3)",
					 paused ? "PAUSED" : "RUN", automatic ? "AUTO" : "HELD", physicsUs / 1000);
			YMGUI_Label_SetText(info, text);
		}
		if (!YMGRE_DemoHost_Step(&host, smoke ? 0 : 60))
		{
			result = smoke ? 1 : 0;
			break;
		}
	}
done:

	if (ready)
		YMGRE_DemoHost_Destroy(&host);
	for (int i = 0; i < 3; ++i)
		destroy(&s[i]);
	if (smoke)
		printf("%s rope window: changed=%d controls=%d frame=%d\n", result ? "FAIL" : "PASS",
			   changed, tested, frame);
	return result;
}
