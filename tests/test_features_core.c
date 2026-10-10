#include "YMPME.h"
#include "EXT/FLIGHT/YMPME_Flight.h"
#include <stdio.h>
#if PME_ENABLE_STATICS || PME_ENABLE_STATICS3D || PME_ENABLE_SOLID_STATICS
#error "Core must not inherit static analysis"
#endif
#if PME_MODEL_SKIN
#error "Core must not inherit skin contact model"
#endif
#if PME_ENABLE_DYNAMICS || PME_ENABLE_RIGID || PME_ENABLE_JOINT || PME_MODEL_PENDULUM
#error "Core must not inherit rigid/joint/model features"
#endif
#if PME_SENSOR_LASER || PME_ENABLE_QUERY || PME_SENSOR_LIDAR || PME_SENSOR_DEPTH || PME_SENSOR_TACTILE
#error "Core must not inherit spatial sensor/query features"
#endif
#if PME_SENSOR_IMU || PME_SENSOR_ENCODER || PME_SENSOR_FORCE
#error "Core must not inherit sensor features"
#endif
#if PME_MODEL_AIRCRAFT || PME_MODEL_ROPE || PME_MODEL_DRIVE || PME_MODEL_TERRAIN ||                \
	PME_MODEL_SUSPENSION || PME_MODEL_VEHICLE || PME_MODEL_GEAR || PME_MODEL_BELT ||               \
	PME_MODEL_PULLEY
#error "Core-only consumers must not inherit rope features"
#endif
#if PME_ENABLE_FLIGHT || PME_FLIGHT_ROTORS || PME_FLIGHT_AERODYNAMICS
#error "Core-only consumers must not inherit optional flight features"
#endif
int main(void)
{
	printf("CORE_ABI body=%zu desc=%zu world=%zu\n", sizeof(pme_body), sizeof(pme_body_desc),
		   sizeof(pme_world));
	return 0;
}
