/* 三种轻量车辆预设：履带、三轮、四轮；组合 DRIVE/TERRAIN/SUSPENSION。
 * Init 读取地面后初始化状态，不保存terrain指针；调用方在每步提供有效高度场。
 * SI单位，姿态rad；throttle/steer会限制在[-1,1]，brake非0制动，0<dt<=1/60s。
 * Step 成功1失败0，失败不保证整体回滚；实例和地形不允许无保护并发改写。
 * worldLimit=0不限制平面范围；正值由应用指定。预设限制pitch/roll在±0.6rad。
 * 状态/轮子支承数组供显示层直接读取，无GRE类型和堆分配。
 * 非通用车辆刚体/翻车/轮胎动力学；旋转惯量、弹簧参数须保持有效正值。 */
#ifndef YMPME_VEHICLE_H
#define YMPME_VEHICLE_H
#include "../../CORE/YMPME_ConvexMesh.h"
#include "../DRIVE/YMPME_Drive.h"
#include "../TERRAIN/YMPME_Terrain.h"
#define PME_VEHICLE_COUNT 3
#define PME_SUPPORT_MAX 6
#define PME_TRACK_LOOP_LENGTH (2.8f + 2 * 3.14159265359f * .25f)
#ifdef __cplusplus
extern "C"
{
#endif
	typedef enum pme_vehicle_kind_
	{
		PME_VEH_TRACKED,
		PME_VEH_TRICYCLE,
		PME_VEH_FOUR_WHEEL
	} pme_vehicle_kind;
	typedef struct pme_wheel_support_
	{
		float x, z, radius, rest, travel;
		float compression, length, force, groundHeight;
		pme_vec3 position, mount;
		int grounded;
	} pme_wheel_support;
	typedef struct pme_vehicle_
	{
		pme_vehicle_kind kind;
		pme_drive drive;
		float worldLimit; /* 0=不限制；正值限定 x/z 于 [-worldLimit,+worldLimit]m，由应用配置。 */
		float y, vy, pitch, roll, pitchRate, rollRate;
		float mass, stiffness, damping, inertiaPitch, inertiaRoll;
		float speed, steering, wheelAngle[PME_SUPPORT_MAX], wheelSteer[PME_SUPPORT_MAX];
		float yawRate, vx, vz, trackDistance[2];
		pme_wheel_support support[PME_SUPPORT_MAX];
		int supportCount, contactCount;
	} pme_vehicle;
	pme_rotation YMPME_Vehicle_Rotation(const pme_vehicle *v);
	void YMPME_Vehicle_Init(pme_vehicle *v, pme_vehicle_kind kind, float x, float z,
							const pme_terrain *terrain);
	int YMPME_Vehicle_Step(pme_vehicle *v, const pme_terrain *terrain, float throttle, float steer, int brake,
						   float dt);
	void YMPME_Vehicle_UpdateSupports(pme_vehicle *v, const pme_terrain *terrain);
#ifdef __cplusplus
}
#endif
#endif
