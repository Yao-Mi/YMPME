/* 编译期扩展开关；CMake 的可选目标会向消费者传递对应定义。
 * 手工构建须统一设置宏，并只编译选中的源文件。核心默认不启用扩展。 */
#ifndef YMPME_FEATURES_H
#define YMPME_FEATURES_H
#ifndef PME_ENABLE_DYNAMICS
#define PME_ENABLE_DYNAMICS 0
#endif
#if PME_ENABLE_DYNAMICS != 0 && PME_ENABLE_DYNAMICS != 1
#error "PME_ENABLE_DYNAMICS must be 0 or 1"
#endif
#ifndef PME_ENABLE_RIGID
#define PME_ENABLE_RIGID 0
#endif
#if PME_ENABLE_RIGID != 0 && PME_ENABLE_RIGID != 1
#error "PME_ENABLE_RIGID must be 0 or 1"
#endif
#ifndef PME_ENABLE_JOINT
#define PME_ENABLE_JOINT 0
#endif
#if PME_ENABLE_JOINT != 0 && PME_ENABLE_JOINT != 1
#error "PME_ENABLE_JOINT must be 0 or 1"
#endif
#ifndef PME_MODEL_PENDULUM
#define PME_MODEL_PENDULUM 0
#endif
#if PME_MODEL_PENDULUM != 0 && PME_MODEL_PENDULUM != 1
#error "PME_MODEL_PENDULUM must be 0 or 1"
#endif
#if PME_ENABLE_DYNAMICS && !PME_ENABLE_JOINT
#error "DYNAMICS requires JOINT"
#endif
#if PME_ENABLE_JOINT && !PME_ENABLE_RIGID
#error "JOINT requires RIGID"
#endif
#if PME_MODEL_PENDULUM && !PME_ENABLE_JOINT
#error "PENDULUM requires JOINT"
#endif
#ifndef PME_ENABLE_QUERY
#define PME_ENABLE_QUERY 0
#endif
#if PME_ENABLE_QUERY != 0 && PME_ENABLE_QUERY != 1
#error "PME_ENABLE_QUERY must be 0 or 1"
#endif
#ifndef PME_SENSOR_LASER
#define PME_SENSOR_LASER 0
#endif
#if PME_SENSOR_LASER != 0 && PME_SENSOR_LASER != 1
#error "PME_SENSOR_LASER must be 0 or 1"
#endif
#ifndef PME_SENSOR_LIDAR
#define PME_SENSOR_LIDAR 0
#endif
#if PME_SENSOR_LIDAR != 0 && PME_SENSOR_LIDAR != 1
#error "PME_SENSOR_LIDAR must be 0 or 1"
#endif
#ifndef PME_SENSOR_DEPTH
#define PME_SENSOR_DEPTH 0
#endif
#if PME_SENSOR_DEPTH != 0 && PME_SENSOR_DEPTH != 1
#error "PME_SENSOR_DEPTH must be 0 or 1"
#endif
#ifndef PME_SENSOR_TACTILE
#define PME_SENSOR_TACTILE 0
#endif
#if PME_SENSOR_TACTILE != 0 && PME_SENSOR_TACTILE != 1
#error "PME_SENSOR_TACTILE must be 0 or 1"
#endif
#ifndef PME_SENSOR_IMU
#define PME_SENSOR_IMU 0
#endif
#if PME_SENSOR_IMU != 0 && PME_SENSOR_IMU != 1
#error "PME_SENSOR_IMU must be 0 or 1"
#endif
#ifndef PME_SENSOR_ENCODER
#define PME_SENSOR_ENCODER 0
#endif
#if PME_SENSOR_ENCODER != 0 && PME_SENSOR_ENCODER != 1
#error "PME_SENSOR_ENCODER must be 0 or 1"
#endif
#ifndef PME_SENSOR_FORCE
#define PME_SENSOR_FORCE 0
#endif
#if PME_SENSOR_FORCE != 0 && PME_SENSOR_FORCE != 1
#error "PME_SENSOR_FORCE must be 0 or 1"
#endif
#ifndef PME_MODEL_AIRCRAFT
#define PME_MODEL_AIRCRAFT 0
#endif
#if PME_MODEL_AIRCRAFT != 0 && PME_MODEL_AIRCRAFT != 1
#error "PME_MODEL_AIRCRAFT must be 0 or 1"
#endif
#ifndef PME_MODEL_DRIVE
#define PME_MODEL_DRIVE 0
#endif
#if PME_MODEL_DRIVE != 0 && PME_MODEL_DRIVE != 1
#error "PME_MODEL_DRIVE must be 0 or 1"
#endif
#ifndef PME_MODEL_TERRAIN
#define PME_MODEL_TERRAIN 0
#endif
#if PME_MODEL_TERRAIN != 0 && PME_MODEL_TERRAIN != 1
#error "PME_MODEL_TERRAIN must be 0 or 1"
#endif
#ifndef PME_MODEL_SUSPENSION
#define PME_MODEL_SUSPENSION 0
#endif
#if PME_MODEL_SUSPENSION != 0 && PME_MODEL_SUSPENSION != 1
#error "PME_MODEL_SUSPENSION must be 0 or 1"
#endif
#ifndef PME_MODEL_VEHICLE
#define PME_MODEL_VEHICLE 0
#endif
#if PME_MODEL_VEHICLE != 0 && PME_MODEL_VEHICLE != 1
#error "PME_MODEL_VEHICLE must be 0 or 1"
#endif
#ifndef PME_MODEL_GEAR
#define PME_MODEL_GEAR 0
#endif
#if PME_MODEL_GEAR != 0 && PME_MODEL_GEAR != 1
#error "PME_MODEL_GEAR must be 0 or 1"
#endif
#ifndef PME_MODEL_BELT
#define PME_MODEL_BELT 0
#endif
#if PME_MODEL_BELT != 0 && PME_MODEL_BELT != 1
#error "PME_MODEL_BELT must be 0 or 1"
#endif
#ifndef PME_MODEL_PULLEY
#define PME_MODEL_PULLEY 0
#endif
#if PME_MODEL_PULLEY != 0 && PME_MODEL_PULLEY != 1
#error "PME_MODEL_PULLEY must be 0 or 1"
#endif
#ifndef PME_MODEL_ROPE
#define PME_MODEL_ROPE 0
#endif
#if PME_MODEL_ROPE != 0 && PME_MODEL_ROPE != 1
#error "PME_MODEL_ROPE must be 0 or 1"
#endif
#ifndef PME_ENABLE_FLIGHT
#define PME_ENABLE_FLIGHT 0
#endif
#ifndef PME_FLIGHT_ROTORS
#define PME_FLIGHT_ROTORS 0
#endif
#ifndef PME_FLIGHT_AERODYNAMICS
#define PME_FLIGHT_AERODYNAMICS 0
#endif
#if (PME_ENABLE_FLIGHT != 0 && PME_ENABLE_FLIGHT != 1) ||                                          \
	(PME_FLIGHT_ROTORS != 0 && PME_FLIGHT_ROTORS != 1) ||                                          \
	(PME_FLIGHT_AERODYNAMICS != 0 && PME_FLIGHT_AERODYNAMICS != 1)
#error "YMPME feature macros must be 0 or 1"
#endif
#if !PME_ENABLE_FLIGHT && (PME_FLIGHT_ROTORS || PME_FLIGHT_AERODYNAMICS)
#error "Flight submodules require PME_ENABLE_FLIGHT=1"
#endif
#ifndef PME_MODEL_SKIN
#define PME_MODEL_SKIN 0
#endif
#if PME_MODEL_SKIN != 0 && PME_MODEL_SKIN != 1
#error "PME_MODEL_SKIN must be 0 or 1"
#endif
#ifndef PME_MODEL_SURFACE_ATLAS
#define PME_MODEL_SURFACE_ATLAS 0
#endif
#if PME_MODEL_SURFACE_ATLAS != 0 && PME_MODEL_SURFACE_ATLAS != 1
#error "PME_MODEL_SURFACE_ATLAS must be 0 or 1"
#endif
#ifndef PME_ENABLE_SOFTBODY
#define PME_ENABLE_SOFTBODY 0
#endif
#if PME_ENABLE_SOFTBODY != 0 && PME_ENABLE_SOFTBODY != 1
#error "PME_ENABLE_SOFTBODY must be 0 or 1"
#endif
#endif
