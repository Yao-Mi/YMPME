# 刚体、关节与接触共同求解

`YMPME_ENABLE_DYNAMICS=ON` 提供 `YMPME::dynamics`，依赖方向为 `DYNAMICS → JOINT → RIGID → CORE`。模块默认关闭，无 GRE、SKE、传感器、文件或堆分配依赖。启用时自动构建必要的 JOINT/RIGID，基础 Body/World 的布局及旧行为保持不变。

## 使用

应用持有刚体、关节、碰撞体和接触工作数组；以 `YMPME_Dynamics_Default()` 初始化世界，再绑定这些数组及数量。使用 `YMPME_Dynamics_Collider(bodyIndex, type)` 初始化碰撞体默认参数。零关节也可使用；同一刚体可被多个碰撞体引用，各形状通过 `localPosition` 和 `localOrientation` 设置相对刚体质心的局部位姿。默认零偏移、单位旋转；全零四元数也按单位旋转处理以兼容零初始化。非零四元数必须为有限单位四元数。

```c
pme_dynamics world = YMPME_Dynamics_Default();
world.bodies = bodies; world.bodyCount = bodyCount;
world.joints = joints; world.jointCount = jointCount;
world.colliders = colliders; world.colliderCount = colliderCount;
world.contacts = contacts; world.contactCapacity = contactCapacity;
/* 外部每步更新电机/加载。检查返回值。 */
pme_result result = YMPME_Dynamics_Step(&world, 1.f / 240, nextTime);
```

步进顺序：前置校验和几何检测 → 外力更新速度 → 准备关节/接触行 → 交替迭代关节与接触冲量 → 每个刚体积分一次 → 输出关节、接触、运动反馈。不要再对这些物体调用旧 World、Joint_Step、Rigid_Step 或 AIRCRAFT 的积分器。

关节公开了 ValidateStep / Prepare / SolveIteration / Finish 分阶段接口，原 `Joint_Step` 复用相同实现。支持通过电机力矩/速度上限产生接触反作用，而不是事后覆写物体位置。

## 碰撞与摩擦

支持球、旋转盒子、闭合凸 Mesh 的所有相互组合，以及它们与固定平面的接触。有限形状先用世界 AABB 排除，再进入精检测；Mesh 包围盒包含局部中心偏移。球/凸体使用最近三角形特征和内外判定，多面体复用 CORE SAT（双方全部面法线与边叉积）。平面为本地 y=0、+Y 朝外的单侧静态半空间。

碰撞体设置 `type=PME_COLLIDER_MESH`、`mesh=&validatedMesh`。网格必须先通过 `YMPME_ConvexMesh_Init`，顶点、索引和预计算缓存由调用方持有且保持有效、不可修改；步进不重复做完整凸性/拓扑验证。网格顶点位于碰撞形状的局部坐标系，通过碰撞体局部位姿变换到刚体坐标系；刚体质心、惯量主轴和惯量仍由应用提供；没有隐式缩放、自动凸包生成或凹网格求解。

多面体接触使用参考面/入射面三角裁剪，相邻三角的重复见证点去重后保留最多四个覆盖区域的代表点。边接触使用支撑边/顶点间最近见证点；球保持单点。Mesh/平面也缩减到最多四点，盒子/平面沿用原顶点路径。代表点不是材料接触面积。适合低面数凸碰撞网格；目前没有窄相位空间树。

法向冲量包含偏心施力的惯量响应；两条切向约束联合更新并投影到三维库仑摩擦圆盘。每步累计冲量避免迭代次数重复放大摩擦上限，恢复系数使用双方较小值，摩擦系数使用几何平均值。

同体和固定体/固定体跳过。category/mask 双向匹配；相同非零 group 为负则忽略、为正则强制检测。动态平面和非法描述返回 INVALID_ARGUMENT。尚未接入旧 World 的组配对矩阵和 Sensor 触发器。

接触容量不足返回 CAPACITY_EXCEEDED，不截断物理接触。校验/容量失败不修改刚体、关节、两类缓存或世界时间，但接触工作数组可被改写。调用方不能忽略失败继续推进时钟。

## 反馈与传感器

每个接触保存共同作用点、A→B 法向、穿透量、累计法向/切向冲量，以及 `reactionA/B`：冲量/dt 得到的子步平均世界坐标力和力矩。双方的力互为相反；力矩分别关于 `originA/B`（子步开始时的质心），转到同一原点后满足角动量守恒。

共同作用点取两侧几何见证点中点，有限穿透时有离散误差。反馈含误差稳定化，不应解释成精确材料压力。只有需要触觉的部件才把力归集到面片/栅格并使用 SurfaceTactile。普通物体可直接消费接触载荷，不创建 skin、UV 或传感器；法向力/单元面积是平均压力，真实接触面积、弹性变形和压力扩散仍需材料模型。

## 当前范围

固定步长建议 240 Hz、48～64 次迭代，dt 最大 0.05 秒。检测仍为 O(n²) 离散配对；没有通用动态体/关节机构 CCD、休眠、凹网格或自动岛划分；可选球体对静态环境 CCD 见下。接触点匹配缓存与接触冲量热启动可选开启，精检测仍逐步执行。较高速度/高刚度/长链需更小子步并实测稳定性。当前新路径用于增量验证，并未把旧车辆、飞行、皮肤动画 demo 全部迁移为共同动力学。

`tests/test_dynamics.c` 覆盖线/角动量守恒、偏心法向碰撞引发旋转、球滚动、盒子多点支撑、旋转盒子包含接触、过滤、容量失败、非法参数，以及电机与碰撞同一步抵消。完整应用见 [grasp_lab](../../../project_Demo/grasp_lab/README.md)。

`test_convex_dynamics` 覆盖盒子/盒子、盒子/Mesh、Mesh/Mesh 四点承重，旋转 AABB 假阳性，失败回滚、球/Mesh、Mesh/平面和偏移包围盒；600 组确定性随机旋转案例比较接触与 SAT，覆盖面、边特征。

## 碰撞体局部位姿

```c
colliders[0] = YMPME_Dynamics_Collider(0, PME_COLLIDER_BOX);
colliders[0].localPosition = YMPME_Vec3(0.5f, 0, 0);
YMPME_Quaternion_AxisAngle(YMPME_Vec3(0, 0, 1), 0.3f,
                          &colliders[0].localOrientation);
```

变换关系为 `shapePosition = bodyPosition + bodyRotation * localPosition`、`shapeOrientation = bodyOrientation * localOrientation`。球、盒子、凸 Mesh 和静态平面的粗检测、精检测均使用这一位姿；接触力臂和惯量响应始终关于**真实刚体质心**。局部偏移不会移动质心或自动修改惯量。多个碰撞体引用同一刚体即可构造复合碰撞体；它们互不碰撞。复合质量属性仍需应用按质量分布提供，包括必要的平行轴项。

## 接触缓存与跨步热启动

默认 `world.cache=NULL`，沿用原先从零冲量开始的求解流程。需要时附加调用方的独立缓冲：

```c
pme_contact_cache cache;
pme_contact_cache_entry entries[128];
pme_result result = YMPME_Dynamics_CacheInit(&cache, entries, 128);
if (result == PME_OK) world.cache = &cache;
/* cache、entries、world 的其他数组须互不重叠，且持续有效。 */
```

每个成功子步仍重新检测几何。缓存按碰撞体对和双方质心坐标系中的几何见证点匹配，一条旧接触最多匹配一条新接触；法向也须一致。默认 `anchorTolerance=0.02` 米、`normalCosine=0.95`，小尺寸场景应相应缩小匹配距离。按有序碰撞体对归并，点匹配只扫描该对的少量旧接触，不在所有接触之间两两搜索。

匹配后，用 `dt / previousDt` 缩放上步法向/切向冲量，切向重新投影并受当前摩擦圆盘限制，再交给共同求解器迭代修正。步长比例超出 `[0.5,2]` 时自动从零冲量开始。反馈报告完整最终冲量，不把预热冲量重复计入。`cache.hits` 表示上一步匹配并预热的点数；`contacts[i].persistence` 为连续匹配步数，新点为 1。

容量必须覆盖实际接触数；不足返回 `PME_CAPACITY_EXCEEDED`，不推进刚体、关节、时钟或旧缓存。分离的接触在成功步结束时丢弃，不向无接触对象施加旧冲量。更换数组地址/长度或世界时间不连续时自动跳过复用。

调用方直接修改数组时，**瞬移、重置/复用刚体槽位、改动形状/局部位姿、质量/惯量、重排数组**之后应调用 `YMPME_Dynamics_CacheClear(&cache)`。该函数保留缓冲和匹配参数。仅修改摩擦系数不必清空，预热时会按当前系数裁剪；需要完全重新开始时也可主动清空。缓存内容是工作状态，不是可直接序列化的场景快照。

接触缓存仅预热接触行；关节缓存可独立附加（见下），两者均不跳过窄相位检测。四个动态方块、4 次迭代的测试中，3 秒后的总高度误差由约 7.6 mm 降至 2.0 mm；这不是所有机构都能减少迭代或提高性能的保证。摩擦临界状态和关节耦合机构可能改变轨迹，例如当前方块抓取对照的提升高度会变化，因此默认 demo 参数保持原求解路径。

`test_dynamics_extensions` 覆盖各形状组合的局部位姿等价性、复合支承、偏置接触对真实质心的力矩、缓存失败不提交、步长变化、摩擦变化、分离/过滤、跨世界复用与低迭代堆叠。另有 `test_grasp --warm-start`、`test_track_lab --warm-start` 对照抓取、释放、正反转、张紧、电机关闭和零摩擦。

此扩展增加 DYNAMICS 的公开结构字段，使用方需重新编译；旧 CORE Body/World、JOINT/RIGID 布局保持不变。没有堆分配或 GRE/SKE/SENSOR 依赖。


## 同时预热关节与接触

```c
pme_joint_cache jointCache;
pme_joint_cache_entry jointEntries[16];
pme_result result = YMPME_Joint_CacheInit(&jointCache, jointEntries, 16);
if (result == PME_OK) world.jointCache = &jointCache;
/* 继续使用 Dynamics_Step；world.cache 可独立启用或保持 NULL。 */
```

`world.jointCache` 默认为 `NULL`。开启后，在所有关节/接触目标准备完毕后施加旧冲量，保证碰撞恢复速度的计算不受预热影响；迭代结束后保存两类缓存，再统一积分一次。关节缓存容量须覆盖 `world.jointCount`。任一缓存校验/容量失败均不提交物理状态或两类缓存。瞬移、修改质量/惯量、复用刚体槽位时，两类已启用的缓存都应清空。

关节冲量默认衰减到 0.75 再复用；重配失效规则和分阶段接口见 [JOINT 文档](../JOINT/README.md)。缓存额外占用调用方存储并增加匹配、预热和保存开销；是否能用更少迭代达到同等精度，需要在目标场景测量。

`test_grasp --warm-all`、`test_track_lab --warm-all` 同时启用两类缓存；原 `--warm-start` 仍只启用接触缓存。默认 demo 求解参数保持不变。`test_dynamics_extensions` 还验证双缓存共同反馈、任一容量不足时均不推进状态，以及仅开启关节缓存的路径。

## 可选球体 CCD

`world.ccd` 默认 `NULL`。按**刚体索引**提供开关，只让需要防高速穿透的球使用连续检测：

```c
uint8_t enabled[2] = {0, 1}; /* 示例 world.bodyCount=2，仅对 body 1 的球开启 */
pme_ccd_event events[8];    /* 一个球、每步最多 8 次冲击 */
pme_ccd ccd;
pme_result result = YMPME_Dynamics_CCDInit(&ccd, enabled, 2, events, 8);
if (result == PME_OK) world.ccd = &ccd;
```

数组与 `ccd` 都由调用方持有，跨步有效，与世界其他缓冲互不重叠；无堆分配，无 GRE 依赖。关闭 DYNAMICS 时不会编入 CCD。配置为 NULL 时保持原离散路径，指定刚体开关为 0 时不执行扫掠。

**首版范围**：启用的刚体必须是动态刚体，只挂一个球碰撞体，球的 `localPosition` 必须为零，且不能连接启用的关节。违反这些条件返回 `PME_INVALID_ARGUMENT`，不会悄悄退回离散路径。支持球对静态平面、旋转盒子、闭合凸三角 Mesh，也支持静态球；静态形状的本地位姿和碰撞过滤均有效。与其他动态体的碰撞仍走离散接触。旋转非球体、偏心球/复合体、关节机构及动态体之间的连续碰撞尚未实现；本功能不能保证高速齿轮和履带不穿透。

流程是在本步外力与普通关节/接触求解后，使用球的实际求解速度扫掠至步末，找到首次撞击时刻 TOI，推进到撞击点，施加恢复/摩擦冲量，再继续剩余时间。盒子/凸网格先用扩张包围盒排除，再对有限三角面、边和顶点求交；包围盒命中不会直接当成真实碰撞，因此圆角附近不会被粗略扩张盒误挡。球碰撞几何不随自转改变，姿态按各段角速度推进。

采用与 RIGID 相同的半隐式固定步长加载：本步外力/重力先更新速度，扫掠路径按更新后的速度分段直线推进，外力不在每个冲击处重新积分。CCD 用于防止从非穿透状态跨越障碍，已有深度重叠仍由离散稳定化处理。接触点附近会加极小外移容差以避开浮点误差造成的重复零时刻命中。

`ccd.maxImpacts` 默认 8，范围 1～32。事件容量必须至少为 **启用球数 × maxImpacts**，即使本步没有命中也需要预留。这让容量错误能在任何物理状态、缓存或事件提交之前返回 `PME_CAPACITY_EXCEEDED`。单球达到冲击上限后，如果剩余路径仍会撞击，则停在最后安全位姿并增加 `clampedBodies`；保留末次求解速度，剩余位姿推进被保守舍弃。调用方可据此减小 dt 或提高上限，不能把该步当成完整高精度轨迹。

每个成功步重置 `ccd.count` 和 `clampedBodies`。`events[0..count)` 是额外 CCD 冲击，**不在 `world.contacts` 或接触缓存中重复记录**：

- `time`：相对本步起点的撞击时刻（秒），同一球按时序输出，多球按刚体索引分组。
- `normal`：静态碰撞体 A 指向球 B；`normalImpulse` / `tangentImpulse` 为 N·s。
- `point` / `originA` / `originB`：撞击点与撞击时两端质心，世界坐标。
- `reactionA/B`：冲量除以**完整 dt**，为本步平均贡献，力矩关于**撞击时质心**。

刚体运动反馈包含离散接触与所有 CCD 冲击的总速度变化；需要汇总接触力时，应合并离散 contacts 和 CCD events。力矩汇总前需换算到同一参考点。事件平均力不是碰撞瞬间峰值或皮肤压力。瞬移/重置刚体时，已附加的接触和关节缓存仍要按各自规则清空。

`test_ccd` 覆盖薄墙穿透对照、解析 TOI/反弹位置、多次反弹及上限、平面/凸网格/局部位姿、边角命中与近失、外力与摩擦角反馈、过滤、缓存隔离及失败不提交。可视化对照见 [ccd_lab](../../project_Demo/ccd_lab/README.md)。
