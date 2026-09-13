#ifndef YMPME_WORLD_H
#define YMPME_WORLD_H
#include "../OPOBJ/YMPME_Body.h"
#include "YMPME_Collision.h"
#include "YMPME_Feedback.h"
typedef struct pme_world_desc_
{
	pme_vec3 gravity;
	int meshPrecision; /* 是否对 AABB 候选启用凸 Mesh 精检测，默认开启。 */
	float fixedStep;
	unsigned maxSubsteps, solverIterations;
	float penetrationSlop, correctionRate, bounceThreshold;
} pme_world_desc;
typedef struct pme_world_
{
	pme_world_desc config;
	PME_Body bodies; /* 连续槽位；Init 借用，Create 拥有。 */
	size_t capacity, bodyCount;
	double accumulator, droppedTime;
	uint64_t stepCount;
	int ownsMemory;
	uint32_t groupMasks[32]; /* 对称碰撞组配对矩阵，默认全部允许。 */
	pme_collision_stats collisionStats; /* 每次 Step/Advance 清零，再累计实际查询。 */
} pme_world;
typedef pme_world* PME_World;
#ifdef __cplusplus
extern "C" {
#endif
pme_world_desc YMPME_World_DefaultDesc(void);
/** 配置两个组是否碰撞（对称写入），组号 0..31。 */
pme_result YMPME_World_SetGroupCollision(PME_World world,uint32_t groupA,uint32_t groupB,int enabled);
/** 使用该世界的精度选项及组矩阵查询，并累加 collisionStats。 */
int YMPME_World_Collide(PME_World world,const pme_body* a,const pme_body* b,pme_contact* contact);
/** 绑定并清零外部 storage；world/storage 均由调用方释放。
 * 不得对尚未 Destroy/Free 的 world 重复 Init；失败不修改输入。
 */
pme_result YMPME_World_Init(PME_World world, PME_Body storage, size_t capacity, const pme_world_desc* desc);
/** 复制描述到空槽；返回借用指针，Remove/Destroy 后失效，槽位可能复用。 */
PME_Body YMPME_World_AddBody(PME_World world, const pme_body_desc* desc);
pme_result YMPME_World_RemoveBody(PME_World world, PME_Body body);
/** 单步半隐式 Euler + 迭代冲量，dt 必须为有限正数；无动态分配。 */
pme_result YMPME_World_Step(PME_World world, float dt);
/** 可选单步反馈；不改变Body/World布局。有效参数检查先于步进；状态缓冲不足返回错误，接触缓冲截断见dropped。
 * 和Step同样清空外力。反馈加速度包含本步碰撞；接触项为迭代增量，可求和/dt。
 * CORE不积分旋转，动态体的角速度/角加速度有效位不设置。 */
pme_result YMPME_World_StepWithFeedback(PME_World world,float dt,pme_world_feedback *feedback);
/** 使用 fixedStep 累积帧时间；返回执行的子步数，负值为错误。
 * 超出 maxSubsteps 的整步时间丢弃并计入 droppedTime，保留不足一步的余数。
 * Force 对本次全部子步生效；若无子步执行则保留 Force。
 */
int YMPME_World_Advance(PME_World world, float frameTime);
float YMPME_World_InterpolationAlpha(const pme_world* world);
/** 清除世界和槽位；仅用于 Init 世界，Create 世界请使用 Free。 */
pme_result YMPME_World_Destroy(PME_World world);
#ifdef __cplusplus
}
#endif
#endif
