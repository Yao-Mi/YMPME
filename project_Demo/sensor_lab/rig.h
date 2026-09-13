#ifndef SENSOR_LAB_RIG_H
#define SENSOR_LAB_RIG_H
#include "CORE/QUERY/YMPME_Query.h"
#include "SENSOR/LIDAR/YMPME_Lidar.h"
#include "SENSOR/DEPTH/YMPME_Depth.h"
#include "SENSOR/TACTILE/YMPME_Tactile.h"
#include "SENSOR/TACTILE/YMPME_TactileWindow.h"
#include "CONFIG/YMPME_Features.h"
#if PME_SENSOR_LASER
#include "SENSOR/LASER/YMPME_Laser.h"
#endif
#if PME_SENSOR_IMU
#include "SENSOR/IMU/YMPME_IMUDevice.h"
#endif
#if PME_SENSOR_ENCODER
#include "SENSOR/ENCODER/YMPME_EncoderDevice.h"
#endif
#if PME_SENSOR_FORCE
#include "SENSOR/FORCE/YMPME_ForceDevice.h"
#endif
#define LAB_BODIES 12
#define LAB_BEAMS 360
#define LAB_DEPTH_WIDTH 80
#define LAB_DEPTH_HEIGHT 48
#define LAB_CELLS 160
/* Demo-only cache of the existing ray query; geometric endpoints before
 * min-range/noise rejection. Does not change the sensor's public frame. */
typedef struct sensor_lidar_segment_
{
	pme_vec3 origin, end;
	int hit;
} sensor_lidar_segment;
typedef struct sensor_rig_
{
	pme_world world;
	pme_body bodies[LAB_BODIES];
	pme_feedback_state states[LAB_BODIES];
	pme_contact_feedback contacts[256];
	pme_world_feedback feedback;
	pme_world_query query;
	pme_ray_provider provider;
	pme_feedback_state pose;
	pme_sensor_clock clock;
	double time;
	float yaw;
	pme_lidar_desc lidar;
#if PME_SENSOR_LASER
	pme_laser_desc laser;
	pme_laser_sample laserReading;
	uint32_t laserSeed;
#endif
	pme_vec3 beams[LAB_BEAMS];
	pme_lidar_point points[LAB_BEAMS];
	pme_lidar_frame scan;
	sensor_lidar_segment lidarTrace[LAB_BEAMS];
	size_t lidarTraceCount;
	pme_depth_desc depth;
	pme_vec3 rays[LAB_DEPTH_WIDTH * LAB_DEPTH_HEIGHT];
	float pixels[LAB_DEPTH_WIDTH * LAB_DEPTH_HEIGHT];
	pme_depth_frame image;
	pme_tactile_desc tactile;
	pme_taxel cells[LAB_CELLS];
	pme_tactile_frame skin, averagedSkin;
	pme_taxel integrals[LAB_CELLS], averagedCells[LAB_CELLS];
	pme_tactile_window skinWindow;
	unsigned skinSteps;
#if PME_SENSOR_IMU
	pme_imu_device imu;
	pme_imu_reading imuReading;
#endif
#if PME_SENSOR_ENCODER
	pme_encoder_device encoder;
	pme_encoder_sample encoderReading;
#endif
#if PME_SENSOR_FORCE
	pme_force_device force;
	pme_force_reading forceReading;
	int relativeForce;
#endif
	uint32_t lidarSeed, depthSeed;
	int extraLoad, noise, filtered, shortRange;
} sensor_rig;
int SensorRig_Init(sensor_rig *s);
int SensorRig_Step(sensor_rig *s, float sideways, float forward, float turn);
int SensorRig_Sample(sensor_rig *s);
int SensorRig_Zero(sensor_rig *s);
void SensorRig_Drop(sensor_rig *s);
#endif
