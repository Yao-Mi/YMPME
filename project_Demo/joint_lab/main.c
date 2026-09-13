#include "scene.h"
#include "demo_host.h"
#include "YMGUI_Label.h"
#include "YMGUI_Invalidate.h"
#include <inttypes.h>
#include <stdio.h>
#include <string.h>
enum
{
	PAUSE = 1,
	STEP = 2,
	MOTOR = 4,
	LIMIT = 8,
	LOAD = 16,
	AUTO = 32,
	UP = 64,
	DOWN = 128,
	CAMERA = 256,
	WIRE = 512,
	RESET = 1024,
	QUIT = 2048
};
static const YMGRE_DemoKeyBinding keyBindings[] = {
	{YMGRE_KEY_SPACE, 0, PAUSE, 0},
	{'n', 0, STEP, 0},
	{'m', 0, MOTOR, 0},
	{'l', 0, LIMIT, 0},
	{'f', 0, LOAD, 0},
	{'t', 0, AUTO, 0},
	{YMGRE_KEY_UP, 0, UP, 0},
	{YMGRE_KEY_DOWN, 0, DOWN, 0},
	{'c', 0, CAMERA, 0},
	{'v', 0, WIRE, 0},
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
	joint_lab_state state;
	static joint_scene scenes[3];
	YMGRE_DemoHost host;

	GYOBJ images[3], rows[3][6], info;
	const char *titles[] = {"REVOLUTE  /  MOTOR PENDULUM", "FIXED  /  CANTILEVER LOAD",
							"PRISMATIC  /  LINEAR SERVO"};
	int smoke = argc == 2 && !strcmp(argv[1], "--smoke-test"), ready = 0, result = 1;
	int i, k, frame = 0, paused = 0, view = 0, wire = 0, verified = 0, title = 0;
	double accumulator = 0, savedTime = 0;
	double previous;
	uint64_t first[3] = {0};
	if (argc > 1 && !smoke)
	{
		fprintf(stderr, "Usage: %s [--smoke-test]\n", argv[0]);
		return 1;
	}
	if (!JointLab_Init(&state))
		goto done;
	for (i = 0; i < 3; ++i)
		if (!JointScene_Init(&scenes[i], i))
			goto done;
	YMGRE_DemoHost_DefaultScale(1);
	if (!YMGRE_DemoHost_Init(&host, 1200, 720, 40))
	{
		fprintf(stderr, "GRE host: %s\n", YMGRE_DemoHost_LastError());
		goto done;
	}
	ready = 1;
	for (i = 0; i < 3; ++i)
	{
		if (!label(host.context->root, i * 400, 6, 400, titles[i]))
			goto done;
		images[i] = YMGRE_DemoHost_AddTarget(&host, YMGRE_Camera_GetRenderTarget(scenes[i].camera),
											 i * 400, 34, 400, 400);
		if (!images[i])
			goto done;
		for (k = 0; k < 6; ++k)
		{
			rows[i][k] = label(host.context->root, i * 400, 444 + k * 25, 400, "");
			if (!rows[i][k])
				goto done;
		}
	}
	info = label(host.context->root, 0, 608, 1200, "");
	if (!info)
		goto done;
	if (!label(host.context->root, 0, 638, 600,
			   "UP/DOWN Target 15 deg | M Motor | T Auto target") ||
		!label(host.context->root, 600, 638, 600, "L Hinge limits +/-80 deg | F Extra load 30 N") ||
		!label(host.context->root, 0, 667, 600,
			   "SPACE Pause | N Single physics step | HOME Reset") ||
		!label(host.context->root, 600, 667, 600, "C Front/3D | V Wireframe | ESC Exit") ||
		!label(host.context->root, 0, 695, 1200,
			   "Cyan line: requested angle.  Red: joint / stop / load."))
		goto done;
	if (!YMGRE_DemoHost_BindKeys(&host, keyBindings, sizeof keyBindings / sizeof keyBindings[0]))
		goto done;

	previous = YMGRE_DemoHost_Time();

	for (;; ++frame)
	{
		double now = YMGRE_DemoHost_Time();
		double elapsed = smoke ? 1.0 / 60 : (double)(now - previous);
		int flags;
		char text[64];
		previous = now;
		if (!title)
		{

			if (host.context)
			{
				YMGRE_DemoHost_SetTitle(&host,
										"YMPME + GRE | Joint lab - motors, limits and feedback");
				title = 1;
			}
		}
		if (smoke)
		{
			if (frame == 15)
			{
				savedTime = state.pendulum.time;
				if (!YMGRE_DemoHost_InjectKey(&host, YMGRE_KEY_SPACE, 1))
					goto done;
			}
			if (frame == 22)
			{
				if (state.pendulum.time != savedTime || !YMGRE_DemoHost_InjectKey(&host, 'n', 1))
					goto done;
			}
			if (frame == 23 && fabs(state.pendulum.time - savedTime - 1.0 / 240) > 1e-8)
				goto done;
			if (frame == 24 && !YMGRE_DemoHost_InjectKey(&host, YMGRE_KEY_SPACE, 1))
				goto done;
			if (frame == 30 && !YMGRE_DemoHost_InjectKey(&host, 'm', 1))
				goto done;
			if (frame == 35 && !YMGRE_DemoHost_InjectKey(&host, 'l', 1))
				goto done;
			if (frame == 40 && !YMGRE_DemoHost_InjectKey(&host, 'f', 1))
				goto done;
			if (frame == 50 && !YMGRE_DemoHost_InjectKey(&host, 't', 1))
				goto done;
			if (frame == 55 && !YMGRE_DemoHost_InjectKey(&host, YMGRE_KEY_UP, 1))
				goto done;
			if (frame == 60 && !YMGRE_DemoHost_InjectKey(&host, 'c', 1))
				goto done;
			if (frame == 65 && !YMGRE_DemoHost_InjectKey(&host, 'v', 1))
				goto done;
			if (frame == 90)
			{
				if (state.servo || state.limits || !state.extraLoad || !state.automatic || !view ||
					!wire || state.target <= .6f)
					goto done;
				if (fabsf(state.fixedJoint.feedback.reactionB.force.y - 49.62f) > .02f ||
					!state.sample.velocityValid)
					goto done;
				for (i = 0; i < 3; ++i)
					if (first[i] == hash(YMGRE_Camera_GetRenderTarget(scenes[i].camera)))
						goto done;
				verified = 1;
			}
			if (frame == 95 && !YMGRE_DemoHost_InjectKey(&host, YMGRE_KEY_HOME, 1))
				goto done;
			if (frame == 100 && !YMGRE_DemoHost_InjectKey(&host, YMGRE_KEY_ESCAPE, 1))
				goto done;
		}
		flags = YMGRE_DemoHost_TakeActions(&host);
		if (flags & QUIT)
		{
			result = smoke ? !(verified && state.servo && state.limits && !state.extraLoad &&
							   !state.automatic)
						   : 0;
			break;
		}
		if (flags & RESET)
		{
			if (!JointLab_Init(&state))
				goto done;
			paused = 0;
			accumulator = 0;
		}
		if (flags & PAUSE)
		{
			paused = !paused;
			accumulator = 0;
		}
		if (flags & MOTOR)
			state.servo = !state.servo;
		if (flags & LIMIT)
			state.limits = !state.limits;
		if (flags & LOAD)
			state.extraLoad = !state.extraLoad;
		if (flags & AUTO)
			state.automatic = !state.automatic;
		if (flags & UP)
			state.target = YMPME_Clamp(state.target + .261799388f, -3, 3);
		if (flags & DOWN)
			state.target = YMPME_Clamp(state.target - .261799388f, -3, 3);
		if (flags & CAMERA)
			view = !view;
		if (flags & WIRE)
			wire = !wire;
		if (paused && (flags & STEP))
		{
			if (!JointLab_Step(&state))
				goto done;
		}
		if (!paused)
		{
			accumulator += fmin(elapsed, .1);
			while (accumulator >= 1.0 / 240)
			{
				if (!JointLab_Step(&state))
					goto done;
				accumulator -= 1.0 / 240;
			}
		}
		for (i = 0; i < 3; ++i)
		{
			const pme_joint_feedback *f = i == 0   ? &state.pendulum.joint.feedback
										  : i == 1 ? &state.fixedJoint.feedback
												   : &state.sliderJoint.feedback;
			JointScene_Render(&scenes[i], &state, view, wire);
			YMGUI_Obj_Invalidate(images[i]);
			if (!frame)
				first[i] = hash(YMGRE_Camera_GetRenderTarget(scenes[i].camera));
			if (i == 0)
			{
				float target =
					state.automatic ? .95f * sinf((float)state.pendulum.time * .7f) : state.target;
				snprintf(text, sizeof(text), "Angle %+.1f / target %+.1f deg",
						 f->coordinate * 57.29578f, target * 57.29578f);
				YMGUI_Label_SetText(rows[i][0], text);
				snprintf(text, sizeof(text), "Encoder %5u | accumulated %" PRId64,
						 (unsigned)state.sample.angle, state.accumulatedTicks);
				YMGUI_Label_SetText(rows[i][1], text);
				snprintf(text, sizeof(text), "Speed %+.2f rad/s | alpha %+.2f", f->speed,
						 state.pendulum.body.feedback.angularAcceleration.z);
				YMGUI_Label_SetText(rows[i][2], text);
				snprintf(text, sizeof(text), "Motor %+.2f N.m | support Fy %+.2f N", f->motorEffort,
						 f->reactionB.force.y);
				YMGUI_Label_SetText(rows[i][3], text);
				snprintf(text, sizeof(text), "Total joint torque at COM: %+.2f N.m",
						 f->reactionB.torque.z);
				YMGUI_Label_SetText(rows[i][4], text);
				snprintf(text, sizeof(text), "Motor %s | limits %s | error %.3f mm",
						 state.servo ? "ON" : "OFF", state.limits ? "ON" : "OFF",
						 f->anchorError * 1000);
				YMGUI_Label_SetText(rows[i][5], text);
			}
			else if (i == 1)
			{
				YMGUI_Label_SetText(rows[i][0], "All six degrees of freedom constrained");
				snprintf(text, sizeof(text), "Mass 2 kg | extra load %d N",
						 state.extraLoad ? 30 : 0);
				YMGUI_Label_SetText(rows[i][1], text);
				snprintf(text, sizeof(text), "Support force Fy: %+.2f N", f->reactionB.force.y);
				YMGUI_Label_SetText(rows[i][2], text);
				snprintf(text, sizeof(text), "Support moment: %+.2f N.m",
						 f->reactionA.torque.z + .45f * f->reactionA.force.y +
							 1.8f * f->reactionA.force.x);
				YMGUI_Label_SetText(rows[i][3], text);
				snprintf(text, sizeof(text), "Load COM acceleration: %.3f m/s2",
						 state.fixedBody.feedback.acceleration.y);
				YMGUI_Label_SetText(rows[i][4], text);
				snprintf(text, sizeof(text), "Anchor %.3f mm | rotation %.4f deg",
						 f->anchorError * 1000, f->angularError * 57.29578f);
				YMGUI_Label_SetText(rows[i][5], text);
			}
			else
			{
				snprintf(text, sizeof(text), "Travel %+.3f m | target %+.3f m", f->coordinate,
						 sinf((float)state.pendulum.time * .8f));
				YMGUI_Label_SetText(rows[i][0], text);
				snprintf(text, sizeof(text), "Speed %+.3f m/s | mass 2 kg", f->speed);
				YMGUI_Label_SetText(rows[i][1], text);
				snprintf(text, sizeof(text), "Drive %+.2f N | limit +/-1.20 m", f->motorEffort);
				YMGUI_Label_SetText(rows[i][2], text);
				snprintf(text, sizeof(text), "Rail support Fy: %+.2f N", f->reactionB.force.y);
				YMGUI_Label_SetText(rows[i][3], text);
				snprintf(text, sizeof(text), "Acceleration X: %+.3f m/s2",
						 state.sliderBody.feedback.acceleration.x);
				YMGUI_Label_SetText(rows[i][4], text);
				snprintf(text, sizeof(text), "Lateral %.3f mm | rotation %.4f deg",
						 f->anchorError * 1000, f->angularError * 57.29578f);
				YMGUI_Label_SetText(rows[i][5], text);
			}
		}
		snprintf(text, sizeof(text), "%s | t %.2f s | physics 240 Hz | encoder uint16",
				 paused ? "PAUSED" : "RUNNING", state.pendulum.time);
		YMGUI_Label_SetText(info, text);
		if (!YMGRE_DemoHost_Step(&host, smoke ? 0 : 60))
		{
			result = smoke ? 1 : 0;
			break;
		}
	}
done:

	if (ready)
		YMGRE_DemoHost_Destroy(&host);
	for (i = 0; i < 3; ++i)
		JointScene_Destroy(&scenes[i]);
	if (smoke)
		printf("JOINT_WINDOW %s (controls, feedback, changing GRE frames)\n",
			   result ? "FAIL" : "PASS");
	return result;
}
