#include "scene.h"
#include "demo_host.h"
#include "YMGUI_Label.h"
#include "YMGUI_Invalidate.h"
#include <stdio.h>
#include <string.h>
enum
{
	PAUSE = 1,
	STEP = 2,
	LOAD = 4,
	FILTER = 8,
	NOISE = 16,
	RANGE = 32,
	CAMERA = 64,
	DROP = 128,
	RESET = 256,
	QUIT = 512,
	ZERO = 1024,
	PROJECTION = 2048,
	DENSITY = 4096,
	XRAY = 8192,
	LASERVIEW = 16384
};
enum
{
	LEFT = 1,
	RIGHT = 2,
	FORWARD = 4,
	BACK = 8,
	TURNLEFT = 16,
	TURNRIGHT = 32
};

static const YMGRE_DemoKeyBinding keyBindings[] = {
	{'a', LEFT, 0, 0},
	{'d', RIGHT, 0, 0},
	{'w', FORWARD, 0, 0},
	{'s', BACK, 0, 0},
	{'q', TURNLEFT, 0, 0},
	{'e', TURNRIGHT, 0, 0},
	{YMGRE_KEY_SPACE, 0, PAUSE, 0},
	{'n', 0, STEP, 0},
	{'f', 0, LOAD, 0},
	{'g', 0, FILTER, 0},
	{'i', 0, NOISE, 0},
	{'r', 0, RANGE, 0},
	{'c', 0, CAMERA, 0},
	{'b', 0, DROP, 0},
	{'z', 0, ZERO, 0},
	{'l', 0, PROJECTION, 0},
	{'k', 0, DENSITY, 0},
	{'x', 0, XRAY, 0},
	{'j', 0, LASERVIEW, 0},
	{YMGRE_KEY_HOME, 0, RESET, 0},
	{YMGRE_KEY_ESCAPE, 0, QUIT, 0},
};

static GYOBJ label(GYOBJ root, int x, int y, int width, const char *text)
{
	GYOBJ p = YMGUI_Creat_Label_Creat(root, x, y, width, 22);
	if (p)
	{
		YMGUI_Label_SetText(p, text);
		YMGUI_Label_SetTextColor(p, GY_ARGB(255, 227, 233, 239));
	}
	return p;
}
static uint64_t hash(GRE_RenderTarget t)
{
	size_t i, n = (size_t)t->width * t->height * sizeof(*t->data);
	const unsigned char *b = (const unsigned char *)t->data;
	uint64_t h = UINT64_C(14695981039346656037);
	for (i = 0; i < n; ++i)
		h = (h ^ b[i]) * UINT64_C(1099511628211);
	return h;
}
int main(int argc, char **argv)
{
	static sensor_scene s;
	YMGRE_DemoHost host;

	GYOBJ images[4], rows[3][5], info, laserInfo, laserMode;
	int smoke = argc == 2 && !strcmp(argv[1], "--smoke-test"), ready = 0, result = 1, frame = 0,
		paused = 0, view = 0, verified = 0, title = 0;
	int i, j;
	double accumulator = 0, saved = 0;
	uint64_t first[3] = {0};
	double previous;
	if (argc > 1 && !smoke)
	{
		fprintf(stderr, "Usage: %s [--smoke-test]\n", argv[0]);
		return 1;
	}
	if (!SensorScene_Init(&s))
		goto done;
	if (smoke && !SensorScene_CheckHandedness(&s))
		goto done;
	YMGRE_DemoHost_DefaultScale(1);
	if (!YMGRE_DemoHost_Init(&host, 1200, 800, 40))
		goto done;
	ready = 1;
	if (!label(host.context->root, 0, 6, 400, "WORLD / SENSOR RIG") ||
		!label(host.context->root, 400, 6, 400, "2D LIDAR / 360 DEGREES") ||
		!label(host.context->root, 800, 6, 400, "DEPTH / PINHOLE CAMERA"))
		goto done;
	images[0] =
		YMGRE_DemoHost_AddTarget(&host, YMGRE_Camera_GetRenderTarget(s.camera), 0, 34, 400, 400);
	images[1] =
		YMGRE_DemoHost_AddTarget(&host, YMGRE_Camera_GetRenderTarget(s.radar), 400, 34, 400, 400);
	images[2] =
		YMGRE_DemoHost_AddTarget(&host, YMGRE_Camera_GetRenderTarget(s.depth), 800, 34, 400, 240);
	if (!label(host.context->root, 800, 281, 400, "TACTILE / CELL-AVERAGED PRESSURE"))
		goto done;
	images[3] =
		YMGRE_DemoHost_AddTarget(&host, YMGRE_Camera_GetRenderTarget(s.skin), 800, 312, 400, 120);
	for (i = 0; i < 4; ++i)
		if (!images[i])
			goto done;
	for (i = 0; i < 3; ++i)
		for (j = 0; j < 5; ++j)
		{
			rows[i][j] = label(host.context->root, i * 400, 444 + j * 25, 400, "");
			if (!rows[i][j])
				goto done;
		}
	info = label(host.context->root, 0, 581, 1200, "");
	if (!info)
		goto done;
	if (!label(host.context->root, 0, 615, 600, "WASD Move sensor | Q/E Turn | C Top/3D view") ||
		!label(host.context->root, 600, 615, 600, "R Lidar range 3/12 m | I Sensor noise") ||
		!label(host.context->root, 0, 644, 600,
			   "G Ignore gold blocks in ray queries | HOME Reset") ||
		!label(host.context->root, 600, 644, 600, "F Extra 10 N on each load | B Drop loads") ||
		!label(host.context->root, 0, 674, 1200,
			   "Z Zero encoder + force | SPACE Pause | N Step | ESC Exit") ||
		!label(host.context->root, 0, 704, 600, "L Off/points/lines/both | K Display density") ||
		!label(host.context->root, 600, 704, 600,
			   "X Depth test/x-ray | Cyan: geometric laser hits"))
		goto done;
	laserInfo = label(host.context->root, 0, 740, 600, "");
	laserMode = label(host.context->root, 600, 740, 600, "");
	if (!laserInfo || !laserMode ||
		!label(host.context->root, 0, 770, 1200,
			   "Single laser: pink beam / yellow hit | +Z direction | 1 mm resolution"))
		goto done;
	if (!YMGRE_DemoHost_BindKeys(&host, keyBindings, sizeof keyBindings / sizeof keyBindings[0]))
		goto done;

	previous = YMGRE_DemoHost_Time();

	for (;; ++frame)
	{
		double now = YMGRE_DemoHost_Time();
		double elapsed = smoke ? 1.0 / 60 : (double)(now - previous);
		int flags, held;
		float dx, dz, turn;
		char text[64];
		size_t hits = 0;
		float nearest = s.rig.lidar.maxRange, peak = 0;
		previous = now;
		if (!title)
		{

			if (host.context)
			{
				YMGRE_DemoHost_SetTitle(&host, "YMPME + GRE | Sensor lab + single-point laser");
				title = 1;
			}
		}
		if (smoke)
		{
			if (frame == 10 && !YMGRE_DemoHost_InjectKey(&host, 'a', 1))
				goto done;
			if (frame == 20 && !YMGRE_DemoHost_InjectKey(&host, 'a', 0))
				goto done;
			if (frame == 30)
			{
				saved = s.rig.time;
				if (!YMGRE_DemoHost_InjectKey(&host, YMGRE_KEY_SPACE, 1))
					goto done;
			}
			if (frame == 40 && (s.rig.time != saved || !YMGRE_DemoHost_InjectKey(&host, 'n', 1)))
				goto done;
			if (frame == 41 && fabs(s.rig.time - saved - 1.0 / 240) > 1e-8)
				goto done;
			if (frame == 45 && !YMGRE_DemoHost_InjectKey(&host, YMGRE_KEY_SPACE, 1))
				goto done;
			if (frame == 50 && !YMGRE_DemoHost_InjectKey(&host, 'f', 1))
				goto done;
			if (frame == 59 && fabsf(s.rig.skin.totalForce.z + 88.86f) > .2f)
				goto done;
			if (frame == 61 && (!YMGRE_DemoHost_InjectKey(&host, YMGRE_KEY_SPACE, 1) ||
								!YMGRE_DemoHost_InjectKey(&host, 'z', 1)))
				goto done;
			if (frame == 62 && !YMGRE_DemoHost_InjectKey(&host, 'n', 1))
				goto done;
			if (frame == 63)
			{
#if PME_SENSOR_ENCODER
				if (s.rig.encoderReading.angle != 0 || s.rig.encoderReading.velocityValid)
					goto done;
#endif
#if PME_SENSOR_FORCE
				if (!s.rig.relativeForce || s.rig.force.tarePending ||
					fabsf(s.rig.forceReading.sample.wrench.force.y) > .001f)
					goto done;
#endif
#if PME_SENSOR_IMU
				if (!s.rig.imuReading.sample.stamp.valid ||
					fabsf(s.rig.imuReading.sample.specificForce.y - 9.81f) > .01f)
					goto done;
#endif
				if (!s.rig.averagedSkin.stamp.valid ||
					fabsf(s.rig.averagedSkin.totalForce.z + 88.86f) > .2f)
					goto done;
			}
			if (frame == 64 && !YMGRE_DemoHost_InjectKey(&host, YMGRE_KEY_SPACE, 1))
				goto done;
			if (frame == 60 && !YMGRE_DemoHost_InjectKey(&host, 'g', 1))
				goto done;
			if (frame == 65 && !YMGRE_DemoHost_InjectKey(&host, 'i', 1))
				goto done;
			if (frame == 70 && !YMGRE_DemoHost_InjectKey(&host, 'r', 1))
				goto done;
			if (frame == 75 && !YMGRE_DemoHost_InjectKey(&host, 'c', 1))
				goto done;
			if (frame == 80 && !YMGRE_DemoHost_InjectKey(&host, 'b', 1))
				goto done;
			if ((frame == 32 || frame == 34 || frame == 36 || frame == 38) &&
				(!YMGRE_DemoHost_InjectKey(&host, 'l', 1) ||
				 !YMGRE_DemoHost_InjectKey(&host, 'l', 0)))
				goto done;
			if (frame == 33 && s.projection.mode != SENSOR_LIDAR_OFF)
				goto done;
			if (frame == 35 && s.projection.mode != SENSOR_LIDAR_POINTS)
				goto done;
			if (frame == 37 && s.projection.mode != SENSOR_LIDAR_LINES)
				goto done;
			if (frame == 39 && (s.projection.mode != SENSOR_LIDAR_BOTH || s.rig.time != saved))
				goto done;
			if (frame == 82 && !YMGRE_DemoHost_InjectKey(&host, 'k', 1))
				goto done;
			if (frame == 83 && !YMGRE_DemoHost_InjectKey(&host, 'x', 1))
				goto done;
			if ((frame == 85 || frame == 87 || frame == 89 || frame == 91) &&
				(!YMGRE_DemoHost_InjectKey(&host, 'j', 1) ||
				 !YMGRE_DemoHost_InjectKey(&host, 'j', 0)))
				goto done;
			if (frame == 86 && s.laserProjection != SENSOR_LIDAR_OFF)
				goto done;
			if (frame == 88 && s.laserProjection != SENSOR_LIDAR_POINTS)
				goto done;
			if (frame == 90 && s.laserProjection != SENSOR_LIDAR_LINES)
				goto done;
			if (frame == 92 && s.laserProjection != SENSOR_LIDAR_BOTH)
				goto done;
			if (frame == 100)
			{
				if (s.projection.mode != SENSOR_LIDAR_BOTH || s.projection.stride != 15 ||
					s.projection.depthTest || !s.rig.extraLoad || !s.rig.noise || !s.rig.filtered ||
					!s.rig.shortRange || !view || s.rig.bodies[11].state.position.x >= 0)
					goto done;
				if (first[0] == hash(YMGRE_Camera_GetRenderTarget(s.camera)) ||
					first[1] == hash(YMGRE_Camera_GetRenderTarget(s.radar)) ||
					first[2] == hash(YMGRE_Camera_GetRenderTarget(s.depth)))
					goto done;
				verified = 1;
			}
			if (frame == 105 && !YMGRE_DemoHost_InjectKey(&host, YMGRE_KEY_HOME, 1))
				goto done;
			if (frame == 110 && !YMGRE_DemoHost_InjectKey(&host, YMGRE_KEY_ESCAPE, 1))
				goto done;
		}
		flags = YMGRE_DemoHost_TakeActions(&host);
		held = YMGRE_DemoHost_HeldKeys(&host);
		if (flags & QUIT)
		{
			result = smoke ? !(verified && !s.rig.filtered && !s.rig.noise && !s.rig.extraLoad) : 0;
			break;
		}
		if (flags & RESET)
		{
			YMPME_World_Destroy(&s.rig.world);
			if (!SensorRig_Init(&s.rig))
				goto done;
			paused = 0;
			accumulator = 0;
		}
		if (flags & PAUSE)
		{
			paused = !paused;
			accumulator = 0;
		}
		if (flags & LOAD)
			s.rig.extraLoad = !s.rig.extraLoad;
		if (flags & FILTER)
			s.rig.filtered = !s.rig.filtered;
		if (flags & NOISE)
			s.rig.noise = !s.rig.noise;
		if (flags & RANGE)
			s.rig.shortRange = !s.rig.shortRange;
		if (flags & CAMERA)
			view = !view;
		if (flags & PROJECTION)
			s.projection.mode = (s.projection.mode + 1) % 4;
		if (flags & DENSITY)
			s.projection.stride = s.projection.stride == 1 ? 5 : s.projection.stride == 5 ? 15 : 1;
		if (flags & XRAY)
			s.projection.depthTest = !s.projection.depthTest;
		if (flags & LASERVIEW)
			s.laserProjection = (s.laserProjection + 1) % 4;
		if ((flags & ZERO) && !SensorRig_Zero(&s.rig))
			goto done;
		if (flags & DROP)
			SensorRig_Drop(&s.rig);
		if ((flags & (FILTER | NOISE | RANGE)) && !SensorRig_Sample(&s.rig))
			goto done;
		dx = ((held & RIGHT) ? 1.f : 0) - ((held & LEFT) ? 1.f : 0);
		dz = ((held & FORWARD) ? 1.f : 0) - ((held & BACK) ? 1.f : 0);
		turn = ((held & TURNRIGHT) ? 1.f : 0) - ((held & TURNLEFT) ? 1.f : 0);
		if (paused && (flags & STEP) && !SensorRig_Step(&s.rig, dx, dz, turn))
			goto done;
		if (!paused)
		{
			accumulator += fmin(elapsed, .1);
			while (accumulator >= 1.0 / 240)
			{
				if (!SensorRig_Step(&s.rig, dx, dz, turn))
					goto done;
				accumulator -= 1.0 / 240;
			}
		}
		SensorScene_Render(&s, view);
		for (i = 0; i < 4; ++i)
			YMGUI_Obj_Invalidate(images[i]);
		if (!frame)
		{
			first[0] = hash(YMGRE_Camera_GetRenderTarget(s.camera));
			first[1] = hash(YMGRE_Camera_GetRenderTarget(s.radar));
			first[2] = hash(YMGRE_Camera_GetRenderTarget(s.depth));
		}
		for (i = 0; i < LAB_BEAMS; ++i)
			if (s.rig.points[i].valid)
			{
				++hits;
				nearest = fminf(nearest, s.rig.points[i].range);
			}
		for (i = 0; i < LAB_CELLS; ++i)
			peak = fmaxf(peak, s.rig.averagedCells[i].pressure);
		snprintf(text, sizeof(text), "Probe X %+.2f / Z %+.2f m", s.rig.pose.position.x,
				 s.rig.pose.position.z);
		YMGUI_Label_SetText(rows[0][0], text);
		snprintf(text, sizeof(text), "Heading %+.1f deg | height 1.20 m", s.rig.yaw * 57.29578f);
		YMGUI_Label_SetText(rows[0][1], text);
#if PME_SENSOR_IMU
		snprintf(text, sizeof(text), "IMU ay %+.3f m/s2 | wy %+.3f rad/s",
				 s.rig.imuReading.sample.specificForce.y,
				 s.rig.imuReading.sample.angularVelocity.y);
		YMGUI_Label_SetText(rows[0][2], text);
#else
		YMGUI_Label_SetText(rows[0][2], "IMU module disabled");
#endif
#if PME_SENSOR_ENCODER
		snprintf(text, sizeof(text), "Encoder u16 %u | speed %+.3f rad/s",
				 (unsigned)s.rig.encoderReading.angle, s.rig.encoderReading.velocity);
		YMGUI_Label_SetText(rows[0][3], text);
#else
		YMGUI_Label_SetText(rows[0][3], "Encoder module disabled");
#endif
#if PME_SENSOR_FORCE
		snprintf(text, sizeof(text), "Force %s %.2f N | Tz %+.2f Nm",
				 s.rig.relativeForce ? "REL" : "ABS", -s.rig.forceReading.sample.wrench.force.y,
				 s.rig.forceReading.sample.wrench.torque.z);
		YMGUI_Label_SetText(rows[0][4], text);
#else
		YMGUI_Label_SetText(rows[0][4], "Force module disabled");
#endif
		snprintf(text, sizeof(text), "360 beams | 1 deg | %zu valid returns", hits);
		YMGUI_Label_SetText(rows[1][0], text);
		snprintf(text, sizeof(text), "Range %.0f m | nearest %.2f m", s.rig.lidar.maxRange,
				 hits ? nearest : 0);
		YMGUI_Label_SetText(rows[1][1], text);
		snprintf(text, sizeof text, "3D %s | 1/%zu | depth %s",
				 SensorLidar_ModeName(s.projection.mode), s.projection.stride,
				 s.projection.depthTest ? "ON" : "OFF");
		YMGUI_Label_SetText(rows[1][2], text);
		snprintf(text, sizeof(text), "Sample %.2f s | scan rate 20 Hz", s.rig.scan.stamp.time);
		YMGUI_Label_SetText(rows[1][3], text);
		snprintf(text, sizeof(text), "Noise %s | gold blocks %s", s.rig.noise ? "ON" : "OFF",
				 s.rig.filtered ? "IGNORED" : "VISIBLE");
		YMGUI_Label_SetText(rows[1][4], text);
		snprintf(text, sizeof(text), "Depth 80x48 | optical Z | %.0f m far", s.rig.depth.farDepth);
		YMGUI_Label_SetText(rows[2][0], text);
		snprintf(text, sizeof(text), "Tactile force %.2f N | peak %.1f Pa",
				 -s.rig.averagedSkin.totalForce.z, peak);
		YMGUI_Label_SetText(rows[2][1], text);
		snprintf(text, sizeof(text), "Skin 48 Hz | added load %d N", s.rig.extraLoad ? 30 : 0);
		YMGUI_Label_SetText(rows[2][2], text);
		YMGUI_Label_SetText(rows[2][3], "Contact location: AABB ESTIMATE");
		YMGUI_Label_SetText(rows[2][4], "Pressure map has no material spreading");
		snprintf(text, sizeof(text), "%s | time %.2f s | physics/tactile 240 Hz | optical 20 Hz",
				 paused ? "PAUSED" : "RUNNING", s.rig.time);
		YMGUI_Label_SetText(info, text);
#if PME_SENSOR_LASER
		if (s.rig.laserReading.valid)
			snprintf(text, sizeof text, "Single laser %.3f m | 20 Hz | range %.0f m",
					 s.rig.laserReading.distance, s.rig.laser.maxRange);
		else
			snprintf(text, sizeof text, "Single laser %s | 20 Hz",
					 s.rig.laserReading.status == PME_LASER_BLIND		   ? "BLIND ZONE"
					 : s.rig.laserReading.status == PME_LASER_OUT_OF_RANGE ? "OUT OF RANGE"
																		   : "NO RETURN");
		YMGUI_Label_SetText(laserInfo, text);
		snprintf(text, sizeof text, "J Single laser display: %s",
				 SensorLidar_ModeName(s.laserProjection));
		YMGUI_Label_SetText(laserMode, text);
#else
		YMGUI_Label_SetText(laserInfo, "Single laser module disabled");
		YMGUI_Label_SetText(laserMode, "Enable YMPME_SENSOR_LASER");
#endif
		if (!YMGRE_DemoHost_Step(&host, smoke ? 0 : 60))
		{
			result = smoke ? 1 : 0;
			break;
		}
	}
done:

	if (ready)
		YMGRE_DemoHost_Destroy(&host);
	SensorScene_Destroy(&s);
	if (smoke)
		printf("SENSOR_WINDOW %s (360 lidar, depth, tactile window, IMU/encoder/force if enabled, "
			   "zero, handedness, controls)\n",
			   result ? "FAIL" : "PASS");
	return result;
}
