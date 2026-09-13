#ifndef YMPME_BODY_H
#define YMPME_BODY_H
#include "YMPME_OBJ.h"
#ifdef __cplusplus
extern "C" {
#endif
/** 默认 Dynamic 球，半径 0.5 m，质量 1 kg，摩擦 0.5。 */
pme_body_desc YMPME_Body_DefaultDesc(void);
/** 初始化调用方拥有的 body；失败不修改原值。 */
pme_result YMPME_Body_Init(PME_Body body, const pme_body_desc* desc);
/** 累加力（N）；一次 Step 或实际执行子步的 Advance 结束后清空。 */
pme_result YMPME_Body_AddForce(PME_Body body, pme_vec3 force);
/** 瞬时冲量（N·s）；只改变 Dynamic 的速度。 */
pme_result YMPME_Body_AddImpulse(PME_Body body, pme_vec3 impulse);
/** 瞬移同时重置插值起点，避免渲染从旧位置滑过来。 */
pme_result YMPME_Body_SetPosition(PME_Body body, pme_vec3 position);
/** 设置 Mesh 碰撞朝向；仅更新碰撞几何，不积分角速度。 */
pme_result YMPME_Body_SetRotation(PME_Body body, pme_rotation rotation);
pme_vec3 YMPME_Body_InterpolatedPosition(const pme_body* body, float alpha);
#ifdef __cplusplus
}
#endif
#endif
