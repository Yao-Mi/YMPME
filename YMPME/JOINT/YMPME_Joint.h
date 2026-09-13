#ifndef YMPME_JOINT_H
#define YMPME_JOINT_H
#include "../CORE/RIGID/YMPME_Rigid.h"
#define PME_JOINT_WORLD SIZE_MAX
#ifdef __cplusplus
extern "C"
{
#endif
	typedef enum pme_joint_type_
	{
		PME_JOINT_FIXED,
		PME_JOINT_REVOLUTE,
		PME_JOINT_PRISMATIC
	} pme_joint_type;
	typedef enum pme_joint_motor_mode_
	{
		PME_JOINT_MOTOR_OFF,
		PME_JOINT_MOTOR_SPEED,
		PME_JOINT_MOTOR_EFFORT
	} pme_joint_motor_mode;
	typedef struct pme_joint_limit_
	{
		int enabled;
		float lower, upper;
	} pme_joint_limit;
	typedef struct pme_joint_motor_
	{
		pme_joint_motor_mode mode;
		float speed, effort, maxEffort; /* rad/s和N m，或m/s和N；effort受maxEffort限制。 */
	} pme_joint_motor;
	typedef struct pme_joint_feedback_
	{
		double time;
		float interval;
		unsigned valid;
		float coordinate, speed;		 /* hinge: 连续rad；slider: m；固定关节均为0。 */
		float anchorError, angularError; /* 受约束方向的位置/姿态误差，m / rad。 */
		pme_wrench reactionA, reactionB; /* 本步平均总约束力/力矩，世界坐标，分别关于A/B质心。
										  * A=WORLD时reactionA.torque关于世界原点；包括电机与限位。
										  * force互为反向；torque因参考点不同不一定互为反向。 */
		float motorEffort;				 /* 本步实际电机力矩/力，正方向为B沿A关节轴旋转/移动。 */
	} pme_joint_feedback;
	typedef struct pme_joint_row_
	{
		pme_vec3 linearA, angularA, linearB, angularB;
		pme_vec3 responseLinearA, responseAngularA, responseLinearB,
			responseAngularB; /* 每步缓存M^-1 J^T。 */
		float target, lower, upper, impulse, inverseEffectiveMass;
	} pme_joint_row;
	typedef struct pme_joint_
	{
		pme_joint_type type;
		size_t bodyA, bodyB;
		int enabled;
		pme_vec3 localAnchorA, localAnchorB;
		pme_quaternion localFrameA, localFrameB;
		pme_joint_limit limit;
		pme_joint_motor motor;
		pme_joint_feedback feedback;
		float wrappedAngle, coordinate;
		pme_joint_row rows[8];
		unsigned rowCount; /* 固定工作缓存，无堆分配。 */
	} pme_joint;
    /* Optional caller-owned warm-start workspace, one entry per joint slot.
     * Entries are opaque solver state: initialize/clear through the APIs below. */
    typedef struct pme_joint_cache_entry_
    {
        pme_joint_type type;
        size_t bodyA, bodyB;
        int enabled;
        pme_vec3 localAnchorA, localAnchorB, axes[3];
        pme_quaternion localFrameA, localFrameB;
        pme_joint_limit limit;
        pme_joint_motor_mode motorMode;
        unsigned rowCount;
        float impulses[8];
    } pme_joint_cache_entry;
    typedef struct pme_joint_cache_
    {
        pme_joint_cache_entry *entries;
        size_t capacity, count, hits, rowHits;
        float interval, factor; /* factor in [0,1], default .75; scales reused impulses */
        double time;
        const pme_rigid_body *bodies;
        const pme_joint *joints;
        size_t bodyCount;
    } pme_joint_cache;
    pme_result YMPME_Joint_CacheInit(pme_joint_cache *cache, pme_joint_cache_entry *entries,
                                    size_t capacity);
    /* Clear after teleport, body mass/inertia edits or reusing array slots. */
    void YMPME_Joint_CacheClear(pme_joint_cache *cache);
    pme_result YMPME_Joint_ValidateCache(const pme_joint_cache *cache, size_t jointCount);
    /* Staged APIs: validate first; prepare ALL joint/contact targets, then warm once.
     * Store once after solving, BEFORE Rigid_End; Finish remains after integration.
     * startTime is the preceding step end time. No allocation or validation here. */
    void YMPME_Joint_WarmStart(pme_rigid_body *bodies, size_t bodyCount, pme_joint *joints,
                              size_t jointCount, pme_joint_cache *cache, float dt,
                              double startTime);
    void YMPME_Joint_CacheStore(const pme_rigid_body *bodies, size_t bodyCount,
                               const pme_joint *joints, size_t jointCount,
                               pme_joint_cache *cache, float dt, double endTime);
    /* NULL cache exactly delegates to Joint_Step. Failure before solving leaves
     * bodies, joints and cache unchanged. Cache capacity must cover jointCount. */
    pme_result YMPME_Joint_StepCached(pme_rigid_body *bodies, size_t bodyCount,
                                    pme_joint *joints, size_t jointCount,
                                    pme_vec3 gravity, float dt, unsigned iterations,
                                    double endTime, pme_joint_cache *cache);
	/* 初始世界锚点和世界关节坐标系，本地Z轴是铰链/滑动轴。
	 * 捕获当前相对姿态为零点；A可为PME_JOINT_WORLD，B必须是有效刚体下标。
	 * 数组由调用者持有，步进期间不得重排/复用。所有缓冲不能重叠。
	 * 固定刚体mass=0；首版不提供运动学平台。 */
	pme_result YMPME_Joint_Init(pme_joint *joint, pme_joint_type type, const pme_rigid_body *bodies,
								size_t count, size_t bodyA, size_t bodyB, pme_vec3 worldAnchor,
								pme_quaternion worldFrame);
	pme_result YMPME_Joint_SetMotor(pme_joint *joint, pme_joint_motor motor);
	pme_result YMPME_Joint_SetLimit(pme_joint *joint, pme_joint_limit limit);
	/* 分阶段接口供 DYNAMICS/外部接触求解器复用。先 ValidateStep，再对每个
	 * body 调 Rigid_Begin -> Prepare -> 交替 SolveIteration/接触 -> Rigid_End -> Finish。
	 * ValidateStep 不修改状态；其余接口不重复校验，不分配，不自行积分。
	 * Prepare/Finish 每步各一次，Finish 必须在所有冲量求解和积分之后。
	 * 调用期间不得更换数组或修改关节描述；不要再调用 Joint_Step。 */
	pme_result YMPME_Joint_ValidateStep(const pme_rigid_body *bodies, size_t bodyCount,
										const pme_joint *joints, size_t jointCount,
										pme_vec3 gravity, float dt, unsigned iterations,
										double endTime);
	void YMPME_Joint_Prepare(pme_rigid_body *bodies, pme_joint *joints, size_t count, float dt);
	void YMPME_Joint_SolveIteration(pme_rigid_body *bodies, pme_joint *joints, size_t count);
	void YMPME_Joint_Finish(const pme_rigid_body *bodies, pme_joint *joints, size_t count, float dt,
							double endTime);
	/* 求解所有刚体/关节一个固定子步：加载->顺序冲量->积分->反馈。
	 * dt=(0,0.05]；建议1/240, iterations=32，iterations范围1..256。
	 * 外部加载每步清零；无隐藏时间累积、无分配、无渲染/传感器依赖。
	 * 本函数尚不求解旧World的形状碰撞；不要把同一物体交给两个积分器。
	 * 铰链每步相对转动应<pi，连续角由单圈角展开。 */
	pme_result YMPME_Joint_Step(pme_rigid_body *bodies, size_t bodyCount, pme_joint *joints,
								size_t jointCount, pme_vec3 gravity, float dt, unsigned iterations,
								double endTime);
#ifdef __cplusplus
}
#endif
#endif
