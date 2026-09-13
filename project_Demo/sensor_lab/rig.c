#include "rig.h"
#include <string.h>
#define V YMPME_Vec3
static int box(sensor_rig *s, pme_vec3 pos, pme_vec3 size, uint32_t category)
{
	pme_body_desc b = YMPME_Body_DefaultDesc();
	b.type = PME_STATIC;
	b.shape = PME_BOX;
	b.position = pos;
	b.halfExtents = size;
	b.category = category;
	return YMPME_World_AddBody(&s->world, &b) != NULL;
}
void SensorRig_Drop(sensor_rig *s)
{
	int i;
	for (i = 8; i < 11; ++i)
	{
		s->bodies[i].state.position.y = 1.5f + .3f * (i - 8);
		s->bodies[i].state.velocity = V(0, 0, 0);
	}
}
static void pose_rotation(sensor_rig *s)
{
	s->pose.rotation = (pme_rotation){
		{{cosf(s->yaw), 0, -sinf(s->yaw)}, {0, 1, 0}, {sinf(s->yaw), 0, cosf(s->yaw)}}};
}
int SensorRig_Zero(sensor_rig *s)
{
	(void)s;
#if PME_SENSOR_ENCODER
	if (YMPME_EncoderDevice_Zero(&s->encoder, s->yaw) != PME_OK)
		return 0;
#endif
#if PME_SENSOR_FORCE
	YMPME_ForceDevice_TareNext(&s->force);
	s->relativeForce = 1;
#endif
	return 1;
}
static int instruments(sensor_rig *s, const pme_sensor_stamp *stamp)
{
	(void)s;
	(void)stamp;
#if PME_SENSOR_IMU
	s->imu.desc.accelNoise = s->noise ? .02f : 0;
	s->imu.desc.gyroNoise = s->noise ? .001f : 0;
	if (YMPME_IMUDevice_Measure(&s->imu, &s->pose, stamp, &s->imuReading) != PME_OK)
		return 0;
#endif
#if PME_SENSOR_ENCODER
	s->encoder.noise = s->noise ? .0001f : 0;
	if (YMPME_EncoderDevice_Measure(&s->encoder, s->yaw, stamp, &s->encoderReading) != PME_OK)
		return 0;
#endif
#if PME_SENSOR_FORCE
	if (s->world.stepCount)
	{
		pme_force_sample source;
		s->force.desc.forceNoise = s->noise ? .02f : 0;
		s->force.desc.torqueNoise = s->noise ? .005f : 0;
		if (YMPME_ForceSensor_Contacts(&s->feedback, 0, s->states[0].position, &source) != PME_OK ||
			YMPME_ForceDevice_Measure(&s->force, &source, &s->states[0], &s->forceReading) !=
				PME_OK)
			return 0;
	}
#endif
	return 1;
}
static pme_result captureLidar(void *context, const pme_ray *ray, pme_ray_hit *hit)
{
	sensor_rig *s = context;
	if (s->lidarTraceCount >= LAB_BEAMS)
		return PME_CAPACITY_EXCEEDED;
	pme_result result = s->provider.cast(s->provider.context, ray, hit);
	if (result != PME_OK)
		return result;
	sensor_lidar_segment *segment = &s->lidarTrace[s->lidarTraceCount++];
	segment->origin = ray->origin;
	segment->hit = hit->hit;
	segment->end =
		hit->hit ? hit->point
				 : YMPME_VecAdd(ray->origin, YMPME_VecScale(ray->direction, ray->maxDistance));
	return PME_OK;
}
int SensorRig_Sample(sensor_rig *s)
{
	pme_sensor_stamp stamp = {s->time, s->time, 0, 1};
	s->pose.time = s->time;
	s->pose.position = s->bodies[11].state.position;
	pose_rotation(s);
	s->query.filter.mask = s->filtered ? ~UINT32_C(2) : UINT32_MAX;
	s->lidar.maxRange = s->shortRange ? 3 : 12;
	s->lidar.noise = s->noise ? .025f : 0;
	s->depth.noise = s->noise ? .01f : 0;
	if (YMPME_SensorClock_Due(&s->clock, s->time))
	{
		if (YMPME_SensorClock_Stamp(&s->clock, s->time, &stamp) != PME_OK)
			return 0;
	}
	else
		stamp.sequence = s->clock.sequence;
	s->lidarTraceCount = 0;
	pme_ray_provider captured = {captureLidar, s};
	if (YMPME_Lidar_Measure(&s->lidar, s->beams, LAB_BEAMS, &s->pose, &stamp, &captured,
							&s->lidarSeed, &s->scan) != PME_OK)
	{
		s->lidarTraceCount = 0;
		return 0;
	}
	if (YMPME_Depth_Measure(&s->depth, s->rays, LAB_DEPTH_WIDTH * LAB_DEPTH_HEIGHT, &s->pose,
							&stamp, &s->provider, &s->depthSeed, &s->image) != PME_OK)
		return 0;
#if PME_SENSOR_LASER
	s->laser.maxRange = s->lidar.maxRange;
	s->laser.noise = s->noise ? .005f : 0;
	if (YMPME_Laser_Measure(&s->laser, &s->pose, &stamp, &s->provider, &s->laserSeed,
							&s->laserReading) != PME_OK)
		return 0;
#endif
	return 1;
}
int SensorRig_Init(sensor_rig *s)
{
	int i;
	pme_body_desc b = YMPME_Body_DefaultDesc();
	pme_sensor_stamp stamp = {0, 0, 0, 1};
	pme_feedback_state floor = {0};
	memset(s, 0, sizeof(*s));
	if (YMPME_World_Init(&s->world, s->bodies, LAB_BODIES, NULL) != PME_OK)
		return 0;
	if (!box(s, V(0, -.1f, 0), V(4, .1f, 5), 1) || !box(s, V(-4, 1.25f, 0), V(.1f, 1.25f, 5), 1) ||
		!box(s, V(4, 1.25f, 0), V(.1f, 1.25f, 5), 1) ||
		!box(s, V(0, 1.25f, 5), V(4, 1.25f, .1f), 1) ||
		!box(s, V(0, 1.25f, -5), V(4, 1.25f, .1f), 1) || !box(s, V(2, 1, 2), V(.7f, 1, .7f), 2) ||
		!box(s, V(-2, 1, -2), V(.55f, 1, .55f), 2))
		return 0;
	b.type = PME_STATIC;
	b.shape = PME_SPHERE;
	b.radius = .85f;
	b.position = V(-2, .85f, 2);
	if (!YMPME_World_AddBody(&s->world, &b))
		return 0;
	b.type = PME_DYNAMIC;
	b.radius = .24f;
	b.restitution = 0;
	for (i = 0; i < 3; ++i)
	{
		b.position = V((float)i - 1, .24f, 0);
		b.mass = (float)i + 1;
		if (!YMPME_World_AddBody(&s->world, &b))
			return 0;
	}
	b.type = PME_KINEMATIC;
	b.sensor = 1;
	b.radius = .13f;
	b.position = V(0, 1.2f, -2);
	if (!YMPME_World_AddBody(&s->world, &b))
		return 0;
	s->feedback.states = s->states;
	s->feedback.stateCapacity = LAB_BODIES;
	s->feedback.contacts = s->contacts;
	s->feedback.contactCapacity = 256;
	s->query = (pme_world_query){&s->world, YMPME_Query_DefaultFilter()};
	s->query.filter.ignoreBody = 11;
	s->provider = (pme_ray_provider){YMPME_Query_WorldRaycast, &s->query};
#if PME_SENSOR_LASER
	s->laser = YMPME_Laser_DefaultDesc();
	s->laser.minRange = .05f;
	s->laser.mount.offset = V(.22f, 0, 0);
	s->laserSeed = 789;
#endif
	s->lidar = (pme_lidar_desc){YMPME_Sensor_IdentityMount(), .05f, 12, 0, 0};
	s->scan.points = s->points;
	s->scan.capacity = LAB_BEAMS;
	s->depth = (pme_depth_desc){YMPME_Sensor_IdentityMount(),
								LAB_DEPTH_WIDTH,
								LAB_DEPTH_HEIGHT,
								65,
								65,
								40,
								24,
								.05f,
								12,
								0,
								0};
	s->image.depth = s->pixels;
	s->image.capacity = LAB_DEPTH_WIDTH * LAB_DEPTH_HEIGHT;
	s->tactile = (pme_tactile_desc){YMPME_Sensor_IdentityMount(), 16, 10, .5f, 1, .02f};
	s->tactile.mount.offset.y = .1f;
	s->tactile.mount.rotation = (pme_rotation){{{1, 0, 0}, {0, 0, -1}, {0, 1, 0}}};
	s->skin.cells = s->cells;
	s->skin.capacity = LAB_CELLS;
	floor.valid = PME_FEEDBACK_POSE;
	floor.position = s->bodies[0].state.position;
	floor.rotation = YMPME_Rotation_Identity();
	if (YMPME_Tactile_Measure(&s->tactile, &floor, &stamp, NULL, 0, &s->skin) != PME_OK)
		return 0;
	if (YMPME_Lidar_Scan360(s->beams, LAB_BEAMS, LAB_BEAMS) != PME_OK ||
		YMPME_Depth_BuildRays(&s->depth, s->rays, LAB_DEPTH_WIDTH * LAB_DEPTH_HEIGHT) != PME_OK ||
		YMPME_SensorClock_Init(&s->clock, .05, 0) != PME_OK)
		return 0;
	s->averagedSkin.cells = s->averagedCells;
	s->averagedSkin.capacity = LAB_CELLS;
	if (YMPME_Tactile_Measure(&s->tactile, &floor, &stamp, NULL, 0, &s->averagedSkin) != PME_OK ||
		YMPME_TactileWindow_Begin(&s->skinWindow, s->integrals, LAB_CELLS) != PME_OK)
		return 0;
#if PME_SENSOR_IMU
	{
		pme_imu_desc d = {0};
		pme_signal_desc accel = YMPME_Signal_DefaultDesc(), gyro = accel;
		d.mount = YMPME_Sensor_IdentityMount();
		d.gravity = s->world.config.gravity;
		accel.minimum = -78.48f;
		accel.maximum = 78.48f;
		accel.resolution = .001f;
		accel.cutoffHz = 20;
		gyro.minimum = -8;
		gyro.maximum = 8;
		gyro.resolution = .001f;
		gyro.cutoffHz = 20;
		if (YMPME_IMUDevice_Init(&s->imu, &d, &accel, &gyro, 789) != PME_OK)
			return 0;
	}
#endif
#if PME_SENSOR_ENCODER
	if (YMPME_EncoderDevice_Init(&s->encoder, 0, 0, 1, 0, 321) != PME_OK)
		return 0;
#endif
#if PME_SENSOR_FORCE
	{
		pme_force_device_desc d = YMPME_ForceDevice_DefaultDesc();
		d.mount.offset.y = .1f;
		d.force.minimum = -1000;
		d.force.maximum = 1000;
		d.torque.minimum = -1000;
		d.torque.maximum = 1000;
		d.force.resolution = d.torque.resolution = .001f;
		d.force.cutoffHz = d.torque.cutoffHz = 20;
		if (YMPME_ForceDevice_Init(&s->force, &d, 654) != PME_OK)
			return 0;
	}
#endif
	s->pose.valid = 31;
	s->pose.position = s->bodies[11].state.position;
	pose_rotation(s);
	if (!instruments(s, &stamp))
		return 0;
	s->lidarSeed = 123;
	s->depthSeed = 456;
	return SensorRig_Sample(s);
}
int SensorRig_Step(sensor_rig *s, float sideways, float forward, float turn)
{
	const float dt = 1.f / 240;
	int i;
	pme_sensor_stamp stamp;
	pme_vec3 oldPosition = s->bodies[11].state.position, oldVelocity = s->pose.velocity,
			 oldOmega = s->pose.angularVelocity;
	float oldYaw = s->yaw;
	s->yaw = remainderf(s->yaw + turn * dt, 6.28318530718f);
	s->bodies[11].state.position.x =
		YMPME_Clamp(s->bodies[11].state.position.x +
						dt * (sideways * cosf(s->yaw) + forward * sinf(s->yaw)) * 1.5f,
					-3.3f, 3.3f);
	s->bodies[11].state.position.z =
		YMPME_Clamp(s->bodies[11].state.position.z +
						dt * (-sideways * sinf(s->yaw) + forward * cosf(s->yaw)) * 1.5f,
					-4.3f, 4.3f);
	if (s->extraLoad)
		for (i = 8; i < 11; ++i)
			YMPME_Body_AddForce(&s->bodies[i], V(0, -10, 0));
	s->time += dt;
	s->feedback.time = s->time;
	if (YMPME_World_StepWithFeedback(&s->world, dt, &s->feedback) != PME_OK)
		return 0;
	stamp = (pme_sensor_stamp){s->time, s->time, (uint64_t)s->world.stepCount, 1};
	if (YMPME_Tactile_FromContacts(&s->tactile, &s->states[0], &stamp, &s->feedback, 0, &s->skin) !=
		PME_OK)
		return 0;
	s->pose.time = s->time;
	s->pose.interval = dt;
	s->pose.position = s->bodies[11].state.position;
	s->pose.velocity = YMPME_VecScale(YMPME_VecSub(s->pose.position, oldPosition), 1 / dt);
	s->pose.acceleration = YMPME_VecScale(YMPME_VecSub(s->pose.velocity, oldVelocity), 1 / dt);
	s->pose.angularVelocity = V(0, remainderf(s->yaw - oldYaw, 6.28318530718f) / dt, 0);
	s->pose.angularAcceleration =
		YMPME_VecScale(YMPME_VecSub(s->pose.angularVelocity, oldOmega), 1 / dt);
	pose_rotation(s);
	if (!instruments(s, &stamp) || YMPME_TactileWindow_Add(&s->skinWindow, &s->skin, dt) != PME_OK)
		return 0;
	if (++s->skinSteps == 5)
	{
		if (YMPME_TactileWindow_Read(&s->skinWindow, &s->averagedSkin) != PME_OK ||
			YMPME_TactileWindow_Begin(&s->skinWindow, s->integrals, LAB_CELLS) != PME_OK)
			return 0;
		s->skinSteps = 0;
	}
	if (YMPME_SensorClock_Due(&s->clock, s->time) && !SensorRig_Sample(s))
		return 0;
	return 1;
}
