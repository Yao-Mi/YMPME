# YMPME API

## 最小用法

```c
#include "YMPME.h"
pme_world world;
pme_body storage[32];
pme_body_desc desc = YMPME_Body_DefaultDesc();
if (YMPME_World_Init(&world, storage, 32, NULL) != PME_OK) return;
desc.position = YMPME_Vec3(0, 4, 0);
PME_Body ball = YMPME_World_AddBody(&world, &desc);
if (!ball) { YMPME_World_Destroy(&world); return; }
YMPME_Body_AddImpulse(ball, YMPME_Vec3(2, 4, 0));
YMPME_World_Advance(&world, 1.0f/60.0f);
pme_vec3 renderPosition = YMPME_Body_InterpolatedPosition(
    ball, YMPME_World_InterpolationAlpha(&world));
/* 应用将 renderPosition 映射到渲染实例。 */
YMPME_World_Destroy(&world);
```

## 类型与接口

| 接口 | 约定 |
|---|---|
| `YMPME_Body_DefaultDesc()` | 动态球，半径 0.5，质量 1，摩擦 0.5，恢复系数 0 |
| `YMPME_World_DefaultDesc()` | 重力 -9.81 Y，120 Hz，最多追赶 8 步，8 次求解迭代 |
| `YMPME_World_Init(w, storage, capacity, desc)` | desc 可 NULL 使用默认；成功清零外部槽位；失败保持原值 |
| `YMPME_Create_World(capacity, desc)` | 分配世界与固定容量槽位，失败 NULL |
| `YMPME_World_AddBody(w, desc)` | 复制描述，失败（容量满或描述非法）NULL |
| `YMPME_World_RemoveBody(w, body)` | 移除本世界物体；应用必须撤销该指针的所有引用 |
| `YMPME_World_Destroy(w)` | 清空 Init 世界和槽位，不释放外部内存；拒绝 Create 世界 |
| `YMPME_Free_World(w)` | 释放 Create 世界；NULL 安全；不释放 userData |
| `YMPME_Body_AddForce(body, force)` | 累加 N；对一次 Advance 内所有执行的子步生效，执行后清空 |
| `YMPME_Body_AddImpulse(body, impulse)` | N·s；立刻按质量改变速度 |
| `YMPME_Body_SetPosition(body, position)` | 瞬移并同步 previousPosition |
| `YMPME_World_Step(w, dt)` | 精确执行一次指定 dt，清空力；不修改 Advance 的累积时钟 |
| `YMPME_World_Advance(w, frameTime)` | 返回非负子步数，非法参数返回负数；无子步时保留力 |
| `YMPME_World_InterpolationAlpha(w)` | 余量/固定步长，范围 [0,1] |
| `YMPME_Collide(a, b, contact)` | 当前状态重叠查询；接触边界包含；contact 可 NULL |
| `YMPME_Debug_KineticEnergy(w)` | 动态物体平移动能，单位 J |

`PME_OK=0`、`PME_INVALID_ARGUMENT=-1`、`PME_CAPACITY_EXCEEDED=-2`（Mesh 预计算缓存不足）；AddBody 容量不足返回 NULL。

## 生命周期与可修改字段

- World 和 Body 是可栈分配的公开结构，但 `active`、`inverseMass`、容量、计数及内存所有权属于库内部状态，应用不得直接改写。
- `state.velocity` 可在两次物理更新之间修改；`state.position` 推荐通过 SetPosition 修改以重置插值。
- `type`、`mass`、`shape` 等初始化不变量需要通过合法描述重新调用 Body_Init（或移除再添加）；直接改 mass 不会自动更新 inverseMass。
- 摩擦、阻尼、恢复系数等必须保持初始化 API 规定的有限合法范围；槽位不可缩容、外部缓存不可搬动。
- Body 指针在移除后立即失效，后续 AddBody 可能复用地址。首版没有带 generation 的句柄。
- World 不支持并发读写，也不拥有应用资源。不要在渲染同时写入 GRE 对象。

## 时间步与持续力

通常一个世界只选 Step 或 Advance 一种驱动方式。Step 适合应用已拥有固定时钟的情况；Advance 适合普通渲染帧循环。最大追赶次数避免卡顿后无限补帧；丢弃的整步时间累计在 `world.droppedTime`。

Force 是累计量，不是每帧的持续属性。Advance 未产生物理子步时会保留 Force，此时重复 AddForce 会继续累加。需要稳定、与渲染帧率无关的持续力时，由应用固定时钟在每次 Step 前添加；不要每个高频渲染帧重复累加同一持续力。重力由世界每步自动应用。

插值绘制在前一步与当前步之间，默认有约一个物理步的视觉延迟；追求即时显示时传 `alpha=1`。每次实际子步会更新 previousPosition，瞬移时同时重置它。

## 碰撞与触发器

所有形状以 position 为中心。Box 的 halfExtents 为半尺寸，不支持旋转。碰撞法线从 A 指向 B，分离时 A 沿负法线、B 沿正法线移动；完全重合等无唯一法线场景按固定轴顺序选择。

当 `(a.category & b.mask) != 0` 且 `(b.category & a.mask) != 0` 才检测。Sensor 可与 Static 查询接触，但不求解响应；进入/停留/离开事件由应用比较相邻步骤的查询结果。没有回调或内部事件队列。

恢复系数取双方较小值，摩擦取几何平均值。低速恢复系数置零，避免地面上反复微小反弹。

## GRE 适配

下述 `YMPME_GRE_SyncObject` 是源码工程的旧姿态同步接口，不随一体 SDK 预编译交付。需要直接借用 GRE 几何时，使用 0.3.x 的共享接口，见下一节及 [SHARED_MESH.md](SHARED_MESH.md)。

链接 `ympme_gre`，包含 `BRIDGE/YMPME_GRE.h`。创建网格后、第一次 LocalToWorld 前保存一份局部 `gre_vertex4d` 数组，保持该数组不可变且与目标顶点不重叠。

```c
YMPME_GRE_SyncObject(ball, greObject, localVertices, vertexCount,
    20.0f, YMPME_World_InterpolationAlpha(&world));
/* 然后调用 GRE 渲染管线。 */
```

适配器写入主顶点、已存在的高级顶点 base、WorldCoordinate 及局部 AABB；保留法线/切线等高级属性。只支持正的均匀 object.scale，不负责旋转、法线旋转、碰撞尺寸同步、子网格链遍历或资源生命周期。渲染局部网格应以物理形状中心为原点。GRE 的临时普通顶点由渲染管线更新。

## 0.3.x：共享几何与索引位宽

- `PME_Index`：由 `YMPME_INDEX_BITS=16/32` 决定的无符号索引类型；`YMPME_ConvexMesh_Init` 的索引参数使用该类型。
- `pme_mesh_view`：描述借用顶点地址、步长、xyz 偏移和米制缩放，以及连续索引或含索引指针的逐面记录。
- `YMPME_ConvexMesh_InitView`：校验并绑定借用布局，不复制几何、不申请内存；由应用提供面轴、边轴缓存。失败保持输出 mesh 不变，但可能写入工作缓存。
- `YMPME_ConvexMesh_Vertex` / `YMPME_ConvexMesh_Index`：统一读取连续或借用布局。仅在初始化成功、下标有效时调用；不要直接假定共享网格的 `.vertices` / `.indices` 非空。
- `YMPME_GRE_Mesh_Bind` / `PrepareRender` / `Unbind`：应用侧 GRE 原生绑定，源码随 SDK 放在 `examples/gre/`，不编进 PME 静态库。

一体 SDK 包默认 16 位，通过 `YMPME_INDEX_BITS` 选择同名位宽的完整库；源码独立构建默认 32 位。共享 GRE 时两库位宽一致。网格结构发生 ABI 变化，旧消费者需重新编译。几何与缓存必须在使用期间有效且保持不变；支持范围仍为闭合凸三角网格。完整调用、所有权和示例见 [SHARED_MESH.md](SHARED_MESH.md)。

## 0.2.0：凸 Mesh 与组过滤

新增 `PME_CONVEX_MESH`、`YMPME_ConvexMesh_Init`、`YMPME_Body_SetRotation`、`YMPME_Body_GetAABB`、`YMPME_CollideWithOptions`、`YMPME_World_Collide` 和 `YMPME_World_SetGroupCollision`。详细参数、所有权和示例见 [MESH_COLLISION.md](MESH_COLLISION.md)。

World 默认 `meshPrecision=1`；涉及 Mesh 的配对可以切换为 AABB 模式。原有 Sphere/Box 解析精检测保留。`collisionGroup` 为 0..31，`ignoreAssembly` 相同且非零时忽略。公开结构新增字段，使用者应重新编译，使用 DefaultDesc 构造默认描述。

## 可选 ROPE 模型

通过 `YMPME_MODEL_ROPE=ON` 构建 `YMPME::rope`，显式包含 `MODEL/ROPE/YMPME_Rope.h`。Init / InitSpan 使用调用方节点与连接缓冲；Step 自行推进，未注册到 World；GetView 返回 O(1) 借用状态视图。SetAnchor、SetMass、AddImpulse 用于规定运动端点、固定 / 释放节点及载荷输入。

模型不依赖 GRE。`YMPME::rope_gre` 提供 Create / Sync / Rebind / Destroy，初始化时分配一次网格，正常同步不分配。参数、生命周期、精度与边界详见 [ROPE 说明](YMPME/MODEL/ROPE/README.md) 和对应头文件。

## 可复用机构与车辆模型

新增 `MODEL/BELT`、`PULLEY`、`GEAR`、`DRIVE`、`TERRAIN`、`SUSPENSION`、`VEHICLE`。各自显式链接 `YMPME::belt` / `pulley` / `gear` / `drive` / `terrain` / `suspension` / `vehicle` 并包含对应 `YMPME_*.h`。接口、单位、缓冲归属、固定预设限制及初始化要求见各头文件和 [模型总览](YMPME/MODEL/README.md)。

原 Demo 内部的 `Gear_*`、`CarDrive_*`、`Vehicle_*`、`Belt_*`、`Hoist_*` 已改为库的 `YMPME_*` 接口。此前这些是示例私有函数；物理核心和 ROPE API 不变。新模型的结构体包含自己的状态/缓存，不含 GRE 类型。

## AIRCRAFT（可选）

新增 `YMPME_Aircraft_Init`、`YMPME_Aircraft_Step`、`YMPME_Aircraft_Rotate` 与实例状态 `pme_aircraft`，链接 `YMPME::aircraft`。单位/命令范围/所有权/积分约定见 `YMPME/MODEL/AIRCRAFT/YMPME_Aircraft.h`。无人机及老鹰是游戏飞行预设；CORE API/ABI 保持不变。

## 控制反馈与可选传感器

新增 `YMPME_World_StepWithFeedback` / `pme_world_feedback`（调用方缓冲）、`pme_feedback_state` 及增量接触冲量；Body/World 布局不变。AIRCRAFT 新增 `YMPME_Aircraft_GetFeedback` 和绕过内置控制的 `YMPME_Aircraft_StepMotors`；实例增加导数缓存，使用方须重新编译，不与旧二进制混用。IMU/ENCODER/FORCE 默认关闭，接口和单位见 `YMPME/SENSOR/README.md`。

## 16位角度编码器输出

`pme_encoder_sample.angle` 改为 `uint16_t` 单圈码值，0..65535对应65536个等分刻度，360°回绕为0。角度不再是展开的弧度值。`velocity` 仍为rad/s，新增 `velocityValid`（首样本为0）；首样本时间戳和角度有效。Init的0参数现在选择65536刻度，原低精度选项保留（1..65536），输出范围统一。样本和编码器结构发生改变，使用方需重新编译。

## 传感器器件处理

可选 `YMPME_IMUDevice`、`YMPME_EncoderDevice`、`YMPME_ForceDevice` 分别提供量程/滤波/量化、安装零位/方向/噪声、六维偏置/噪声/去皮。旧测量API兼容。`YMPME_TactileWindow` 在外部缓存中按时间平均单元力/压力。链接原有sensor目标即可，见 [SENSOR](YMPME/SENSOR/README.md#可选器件处理接口)。

## 曲面皮肤反馈

`MODEL/SKIN/YMPME_SkinContact.h` 是可选的采样曲面与球形压头接触模型，`SENSOR/TACTILE/YMPME_SurfaceTactile.h` 是独立曲面压力/剪切力读出接口；共享CORE的SI曲面采样协议。两者均无SKE/GRE依赖。见[低模手实验台](project_Demo/hand_skin/README.md)和[接触模型](YMPME/MODEL/SKIN/README.md)。

## 0.3.1：日志诊断

`DEBUG/YMPME_Log.h` 提供 `YMPME_LOG`、`YMPME_REPORT`、`YMPME_ASSERT`，以及 SetLogger、ResetLogger、Log、Report、ResultString、LevelString 和 AssertFail 接口（函数前缀均为 `YMPME_Debug_`）。默认 WARNING/ERROR 输出至 stderr，普通日志不终止进程；断言单独编译控制，默认关闭。源代码详见 `YMPME/DEBUG/README.md`，SDK 中对应文档为 `docs/DEBUG.md`。

日志只补充诊断，原有返回值和状态修改规则保持不变。当前重点覆盖物体、基础世界、凸网格、动力学步进和 GRE 共享绑定错误；不是所有传感器/模型接口都已接入。配置是库级别的，仿真运行前设置，不能并发修改；回调同步消费借用记录，禁止重入日志。

## 0.4.0：版本与步进边界

`YMPME_VERSION_MAJOR/MINOR/PATCH` 和 `YMPME_VERSION_STRING` 描述编译头文件；`YMPME_VersionString()` 返回已链接库的版本字符串。CMake 和 SDK 构建使用同一宏来源，外部程序可比较两者，识别旧库或混用头文件。

RIGID/JOINT/DYNAMICS 的步末时间必须能够严格前进；极小到 `1/dt` 无法有限表示的步长被拒绝，CORE 的固定步长和反馈步进亦检查这一边界。单独 `YMPME_Rigid_Step` 在局部副本求解，结果或导数溢出返回 `PME_INVALID_ARGUMENT` 且保留输入。分阶段接口仍由调用者保证数值范围，不提供任意极端有限输入的全机构回滚保证。

三个静力组件分别为 `YMPME::statics`、`YMPME::statics3d`、`YMPME::solid_statics`，见 `YMPME/STATICS/` 文档。升级到新的 minor 版本或更换索引位宽时重新编译，不能混用不一致的结构布局。
