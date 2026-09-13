# 可裁剪关节模块

JOINT 提供可复用的约束原语；MODEL 负责组装机构；控制器与传感器位于上层。模块依赖方向为 `MODEL → JOINT → CORE/RIGID → CORE`，所有这些物理层均不依赖 GRE。

| 类型 | 锁定方向 | 自由坐标 | 电机单位 |
| --- | --- | --- | --- |
| FIXED | 3 个平移、3 个转动 | 无 | 无 |
| REVOLUTE | 3 个平移、2 个转动 | 沿关节 Z 轴的连续角 rad | N·m、rad/s |
| PRISMATIC | 2 个平移、3 个转动 | 沿关节 Z 轴的位移 m | N、m/s |

## 使用

启用 `YMPME_ENABLE_JOINT=ON`，链接 `YMPME::joint`。自动构建必要的 `YMPME::rigid`，不会打开飞行、其他 Model、传感器或渲染。`YMPME_ENABLE_RIGID=ON` 也可单独启用自由刚体。关闭的源文件不会编译，旧核心 ABI 不变。

```c
pme_rigid_body bodies[2];
pme_joint hinge;
pme_quaternion identity = YMPME_Quaternion_Identity();
YMPME_Rigid_Init(&bodies[0], 0, YMPME_Vec3(0,0,0),
                 YMPME_Vec3(0,2,0), identity);
YMPME_Rigid_Init(&bodies[1], 1, YMPME_Vec3(.1f,.1f,.1f),
                 YMPME_Vec3(0,1,0), identity);
YMPME_Joint_Init(&hinge, PME_JOINT_REVOLUTE, bodies, 2, 0, 1,
                 YMPME_Vec3(0,2,0), identity);
YMPME_Joint_SetLimit(&hinge, (pme_joint_limit){1,-1.4f,1.4f});
YMPME_Joint_SetMotor(&hinge,
    (pme_joint_motor){PME_JOINT_MOTOR_SPEED, 1, 0, 10});
/* 每个固定子步更新time；检查以上Init/Set和以下Step的返回值。 */
YMPME_Joint_Step(bodies, 2, &hinge, 1, YMPME_Vec3(0,-9.81f,0),
                 1.f/240, 32, 1.0/240);
```

`Init` 用世界锚点与世界关节坐标系生成两端本地框架，捕获当前相对姿态作为零位。Z 轴是转轴/滑动轴，可通过 `Quaternion_AxisAngle` 转到任意方向；A 可用 `PME_JOINT_WORLD` 固定在世界，B 必须为有效下标。体和关节数组由调用者管理，求解期间不能删除、重排、复用或重叠缓冲。

电机支持关闭、目标速度和指定力矩/力三种模式。后两者均受 `maxEffort` 限制；关节内部不包含 PID。指定力矩模式适合上层控制器输出真实执行器载荷，目标速度模式适合简单游戏机构。`SetLimit` 支持单轴上下限；把上下限设为同值可锁住该自由度。把 `joint.enabled` 设为 0 可释放整个连接，反馈载荷清零；重新启用会恢复原约束。

## 反馈与传感器

`joint.feedback` 是上一个子步的数值反馈，初始化时 `valid=0`。`coordinate/speed` 为连续角和角速度，或位移和速度；角度传给已有 Encoder 后可得到 0..65535 的 `uint16_t` 输出。

`reactionA/B` 是关节施加给两端的总力/力矩，包含锚点约束、电机、限位和误差稳定化，由本步累计求解冲量除以 dt 得到。力矩为世界坐标、关于各自质心；A 为 WORLD 时关于世界原点。不同参考点的两个力矩不能直接比较正负。`motorEffort` 单独报告实际电机输出，不等于总约束力矩。

例如，要得到 B 端锚点处的纯力矩：`torqueAtAnchor = reactionB.torque - cross(anchor - bodyB.position, reactionB.force)`。反馈冲量的力臂取子步求解时的姿态；高速运动时应保存该步开始的锚点/质心进行换点。固定端可直接按不变坐标换算。转动关节即使电机关闭，关于质心的总力矩也可能非零，因为锚点支承力具有力臂。

刚体的 `feedback` 提供位置、角速度、加速度和角加速度，可接 IMU；`reactionB` 可接现有 ForceSensor 的 Wrench 接口。这是约束反作用真值，不是碰撞接触面的压力分布；手指皮肤阵列仍需接触流形与空间采样模型。

## 求解与范围

使用顺序冲量和速度级误差稳定化；每步缓存约束 Jacobian 与 `M^-1 J^T`，迭代中不重复构建旋转矩阵。不申请内存，不依赖渲染帧率。建议 240 Hz、32 次迭代，多节链可提高到 64；仍然存在有限步长和迭代误差，不是精确解析机构求解器。铰链每步的真实相对转角必须小于 pi，才能正确展开连续角。

首版针对无碰撞机构：尚未把这些通用旋转刚体接入旧 World 的 Mesh/摩擦碰撞求解，也没有运动学平台、球关节、弹性关节和自动断裂。不要将同一物体同时交给 `Joint_Step`、`Rigid_Step`、旧 `World_Step` 或 AIRCRAFT 重复积分。现有齿轮、皮带、滑轮、悬挂等专用 Model 保持原物理方法，后续按实际需要迁移。

`tests/test_joint.c` 验证固定承载、三维动态体反作用、多圈正反转、电机/双边限位、滑动约束、移动参考系动量、多节链和释放。`python3 tests/check_joints.py` 验证 5 种独立裁剪组合、核心 ABI/归档一致以及无堆/渲染/传感器依赖。

## 与碰撞共同求解

可选 [DYNAMICS](../DYNAMICS/README.md) 复用 JOINT 分阶段接口，将关节与球/盒子/凸 Mesh/平面接触交替求解。其最终步进接口代替 Joint_Step，保证每个刚体只积分一次；原无碰撞 Joint_Step 保持兼容。


## 可选关节热启动

原 `YMPME_Joint_Step` 保持从零冲量开始。需要跨步复用时，调用方提供每个关节一个缓存条目：

```c
pme_joint_cache cache;
pme_joint_cache_entry entries[16];
pme_result result = YMPME_Joint_CacheInit(&cache, entries, 16);
if (result == PME_OK)
    result = YMPME_Joint_StepCached(bodies, bodyCount, joints, jointCount,
                                   gravity, dt, iterations, endTime, &cache);
```

`cache` 和 `entries` 须在多次步进间保持有效，并与其他缓冲互不重叠。缓存容量不足以容纳 `jointCount` 时返回 `PME_CAPACITY_EXCEEDED`，校验失败或容量不足均不改动刚体、关节、缓存或时钟。传 `NULL` 缓存等价于原 `Joint_Step`。缓存属于 JOINT，不引入 DYNAMICS、传感器或显示依赖，也无堆分配；原刚体、关节及约束行结构布局不变。

预热使用 `previousImpulse * dt / previousDt * cache.factor`，按当前行上下限裁剪，并由本步迭代继续修正。`factor` 默认 **0.75**，范围 `[0,1]`；`0` 不复用旧冲量，但仍维护缓存。该衰减避免低迭代长链把上一帧误差不断放大；不要假设设为 1 更好。限位/电机仍使用原有的完整约束，反馈由本步最终总冲量计算，不会重复统计预热载荷。

按关节数组槽位 O(n) 匹配。更换数组地址/数量、时间不连续、步长比例超出 `[0.5,2]` 时从零开始；关节类型、两端索引、本地锚点或本地框架改变也会丢弃对应缓存。参考轴相对缓存旋转过大时不复用。关闭的关节不预热；电机模式变更或关闭时不复用旧电机行；限位开关或边界改变时不复用旧限位行。改变速度目标或力上限后由当前求解目标及上下限修正。`hits` 是最近一步复用的关节数，`rowHits` 是复用行数，包括冲量恰为零的行。

瞬移、修改刚体质量/惯量、重置或复用数组槽位后，调用 `YMPME_Joint_CacheClear(&cache)`；它保留缓冲容量及 `factor`。缓存是求解工作状态，不能作为可直接序列化的场景。关节设置在两个步进之间改回原值，无法仅靠快照识别这种重置，也应主动清空。

共同求解器可按顺序调用 `ValidateStep`、`ValidateCache` → `Rigid_Begin` → 准备**所有**关节/接触目标 → `WarmStart`（一次）→ 迭代 → `CacheStore`（积分前）→ `Rigid_End` → `Finish`。分阶段接口不重复验证，也不自行积分；不能在迭代中反复预热。

`test_joint_cache` 覆盖固定支承反馈、铰链/滑块电机与双向限位、禁用与重配、步长/数组/时钟变化、失败不提交和低迭代长链。8 节竖直悬链、240 Hz、4 次迭代、后 5 秒的最大连接误差，由约 1.33 mm 降到 0.33 mm；这只证明该工况的收敛改善，不保证所有机构都能降低迭代数或总耗时。
