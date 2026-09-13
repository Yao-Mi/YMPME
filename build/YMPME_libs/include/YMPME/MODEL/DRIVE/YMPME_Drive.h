/* 差速轮+拖距万向轮运动学。Reset 初始化几何和零状态。
 * 状态由调用方持有，无分配，可直接只读显示；参数采用m、s、rad。
 * targetLeft/Right为轮缘速度m/s；0<dt<=.05s；几何尺寸必须有限且>0。
 * 不含地面碰撞、悬挂或轮胎滑移。Step 非法输入不改状态。 */
/* 平面差速运动学；不代表通用旋转刚体或轮胎接触求解器。 */
#ifndef YMPME_DRIVE_H
#define YMPME_DRIVE_H
#include "../../CORE/YMPME_MathBase.h"
#ifdef __cplusplus
extern "C"
{
#endif
	typedef struct pme_drive_
	{
		float x, z, yaw;   /* Y 向上，yaw=0 朝 +Z，正 yaw 向右转。 */
		float left, right; /* 轮缘速度 m/s。 */
		float leftAngle, rightAngle, casterAngle, casterSpin;
		float track, radius, casterRadius, casterTrail, casterZ;
		float acceleration;
	} pme_drive;
	void YMPME_Drive_Reset(pme_drive *car);
	/** 目标轮速经加速度限制后积分；返回 PME_INVALID_ARGUMENT 时不修改状态。 */
	pme_result YMPME_Drive_Step(pme_drive *car, float targetLeft, float targetRight, float dt);
#ifdef __cplusplus
}
#endif
#endif
