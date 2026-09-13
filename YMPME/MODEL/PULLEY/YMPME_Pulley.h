/* 理想四股承重的 4:1 滑轮组预设。Init 设置8kg负载、0.5kg手柄及初始绳长常数。
 * 坐标向上为正(m)，速度m/s，张力/拉力N，质量kg；结构和全部状态由调用方持有。
 * hold 和质量可在步间改变，质量必须有限且>0；command [-1,1] 为升降指令。
 * Step 接受 0<dt<=0.01 秒，建议 dt<=1/600 秒；非法指令/dt/质量或空指针不改状态。没有绕轮碰撞或绳子脱槽。
 * 此版本保留示例的手柄调速器与 [-.5,.3]m 负载限位，不是任意穿绕拓扑。 */
#ifndef YMPME_PULLEY_H
#define YMPME_PULLEY_H
#include "../../CORE/YMPME_MathBase.h"
#ifdef __cplusplus
extern "C"
{
#endif
	typedef struct pme_pulley_
	{
		double loadY, freeY, loadV, freeV, constant, loadMass, handleMass, tension, pull;
		int hold;
	} pme_pulley;
	void YMPME_Pulley_Init(pme_pulley *h);
	void YMPME_Pulley_Step(pme_pulley *h, double command, double dt);
#ifdef __cplusplus
}
#endif
#endif
