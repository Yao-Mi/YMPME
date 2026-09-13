# AABB、凸 Mesh 与碰撞过滤

## 检测顺序

1. 检查有效物体，排除自己、同装配体、category/mask 不匹配和世界碰撞组矩阵禁止的配对。
2. 生成世界 AABB；不重叠直接返回，不进入形状精检测。
3. Sphere / Box 继续使用原来的解析检测。
4. 涉及 Mesh 的配对：精度开启时使用凸多面体 SAT；关闭时按世界 AABB 接触处理。

`YMPME_Collide` 默认启用 Mesh 精检测，但没有某个 World 的组矩阵上下文。需要世界配置时用 `YMPME_World_Collide`；也可用 `YMPME_CollideWithOptions` 显式传入选项和计数器。World 求解器已经使用同一流程。

```c
world.config.meshPrecision = 1; /* AABB 候选 -> 凸 Mesh 精检测 */
world.config.meshPrecision = 0; /* 涉及 Mesh 的配对采用 AABB 响应 */
```

## 支持的 Mesh

支持**闭合、非退化、凸的三角网格**，包括三角化盒体、凸多面体和圆柱车轮。检测基于传入的实际顶点，而非另建球形代理；多边形圆柱的精度取决于网格分段数。

支持 Mesh–Mesh、Mesh–Box、Mesh–Sphere，含旋转、边角接触和完全包含。SAT 使用双方的面法线及边叉积；球体额外检查到三角形最近特征的方向。接触输出仍为法线和穿透深度，供现有平移求解器使用。

凹网格、开口地形、平面三角片不属于这个凸体接口，初始化会拒绝。凹模型应离线拆成多个闭合凸零件并逐件检测；不会静默把凹模型当作凸包。基础凸体接口不提供自动凸分解、任意三角汤检测或 BVH。这里的基础 World 使用平移求解；三维旋转刚体、接触流形和可选的球体对静态形状 CCD 由 DYNAMICS 接口提供，不能把两个世界的数据结构混用。

## 构建与绑定

```c
pme_convex_mesh mesh;
pme_vec3 faceAxes[TRIANGLE_COUNT];
pme_vec3 edgeAxes[TRIANGLE_COUNT * 3];
/* vertices 是米制局部顶点；indices 声明为 PME_Index[]，每三项一个三角。 */
pme_result result = YMPME_ConvexMesh_Init(&mesh,
    vertices, VERTEX_COUNT, indices, TRIANGLE_COUNT,
    faceAxes, TRIANGLE_COUNT, edgeAxes, TRIANGLE_COUNT * 3);
if (result != PME_OK) { /* 处理非法网格或容量不足 */ }

pme_body_desc desc = YMPME_Body_DefaultDesc();
desc.shape = PME_CONVEX_MESH;
desc.mesh = &mesh;
desc.position = YMPME_Vec3(0, 1, 0);
PME_Body body = YMPME_World_AddBody(&world, &desc);
```

四组数组和 `mesh` 都由应用持有；必须在 Body 使用期间保持有效且不修改，工作数组彼此也不能重叠。初始化验证有限坐标、索引、三角形面积、每条边恰有两个相邻面、凸性和非零体积；可以处理三角形绕序不统一，但相邻面必须共享顶点索引。失败不修改 mesh，但工作数组可能已被写入；不要拿仍被使用的 Mesh 缓存做失败试验。

预计算时去重法线和边方向；检测时无堆分配。绑定多个实例可共享同一不可变 Mesh，但每个 Body 有独立 position 和 rotation。旋转矩阵的三列必须是右手正交单位基，可用 `YMPME_Rotation_Identity` 初始化、`YMPME_Body_SetRotation` 校验设置。连续数组 Init 没有隐式缩放；借用视图 InitView 可用固定的 vertexScale 在读取时换算为米。

## 共享 GRE 网格（0.3.x）

`YMPME_ConvexMesh_InitView` 支持借用带步长的顶点以及逐面索引指针，SAT、射线查询、接触流形和适用的 CCD 路径都通过统一访问器读取。`PME_Index` 随 16/32 位库编译；共享 GRE 时双方位宽一致，不进行索引格式转换。

应用可编译 SDK 的 `examples/gre/YMPME_GREMesh.c`，直接借用 GRE 原始局部几何。显示变换使用 GRE 的工作顶点，仍需独立的碰撞轴缓存和面片工作记录。借用不改变凸体限制，也不自动迁移已有 Demo。绑定、源对象生命周期及释放顺序见 [SHARED_MESH.md](SHARED_MESH.md)。

## 组与装配体过滤

三个条件都必须允许才检测，不存在某一层强行覆盖另一层的规则。

```c
/* 原有类别位掩码：两边都需要匹配。 */
chassis->state.category = 1u;
chassis->state.mask = UINT32_MAX;

/* 世界组矩阵：组号 0..31，默认全部允许，设置自动对称。 */
chassis->state.collisionGroup = 1;
wheel->state.collisionGroup = 2;
YMPME_World_SetGroupCollision(&world, 1, 2, 0); /* 车身组与轮子组不检测 */
YMPME_World_SetGroupCollision(&world, 2, 2, 0); /* 同轮子组之间不检测 */
YMPME_World_SetGroupCollision(&world, 1, 0, 1); /* 允许车身组与环境组 */

/* 整个车内部都要忽略时，给同车零件相同的非零装配体 ID。 */
chassis->state.ignoreAssembly = 100;
wheel->state.ignoreAssembly = 100;
```

`ignoreAssembly=0` 表示没有装配体忽略规则。不同车使用不同 ID，则车辆之间仍能检测。若同车只想忽略某些配对，保持这些 Body 的 `ignoreAssembly=0`，使用组矩阵细分。组矩阵是世界级规则，会作用于所有属于相应组的 Body。

小车示例：环境组 0，车身组 1，驱动轮组 2，万向轮组 3；同车零件统一使用装配体 ID 1。所有车内配对在 AABB 之前跳过，环境配对仍参与检测。

## 小车窗口

运行 `./build/project_Demo/three_wheel_car/demo_three_wheel_car`。默认开启 Mesh 精检测；**H** 切换 Mesh / AABB，切换后回到初始位置，以避免占用体积变化后卡在障碍里。

每个活动零件都由对应的 GRE 局部顶点构造凸碰撞 Mesh，车轮滚转和万向轮转向与绘制共用姿态计算。外部箱体和围栏使用与显示尺寸一致的 Box。原来整车半径 0.86 m 的球形碰撞代理已经移除；根 Body 只保存位置且 mask=0。

小车仍是整体驱动的平面装配体。检测到新姿态侵入障碍时，在旧姿态与候选姿态间回退位置和朝向，防止零件散开。这是离散步内回退，**不是连续碰撞检测**。不计算撞击造成的车身转动，也不处理从一开始就被外部代码瞬移到障碍内部的恢复；此时可按 R 重置。

## 统计与代价

`world.collisionStats` 包含 `filteredPairs`、`aabbTests`、`aabbRejected`、`narrowphaseTests`、`meshTests`。Step / Advance 开始时清零，期间和之后的 World_Collide 查询累加。包含求解迭代或装配体回退时的重复查询，不是唯一配对数量。窗口显示最近物理步的统计。

AABB 只排除当前配对，不提供空间树，因此世界遍历仍为平方级。凸 Mesh SAT 的成本取决于顶点数、独立法线数和边方向数；只在 AABB 重叠后发生。建议使用独立低面数碰撞网格控制成本。
