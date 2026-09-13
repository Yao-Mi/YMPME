# 回归测试

`ctest --test-dir build --output-on-failure` 运行 8 组核心测试；启用 GRE 适配后增加第 9 组。测试不用 assert，因此 Release/NDEBUG 下仍执行校验。

- integration：自由落体、运动学、静态、力/冲量、阻尼、瞬移。
- collision：球/球、球/盒六面、盒内、角点、盒/盒、反序法线。
- response：不等质量弹性碰撞的动量/动能及运动平台推动。
- resting：球/盒长期落地和摩擦减速。
- lifecycle：非法参数、容量、移除、槽位复用、失败不改写。
- fixed_step：直接单步与帧累积一致、插值、超时丢弃、跨子步力语义。
- filtering：Sensor 无响应、双向掩码过滤。
- memory：两处分配失败回滚、容量溢出、步进零分配、释放平衡。
- gre_bridge：真实 GRE 结构体，1000 帧无变换累加，单位/插值/属性/AABB。

可用 `-DYMPME_ENABLE_SANITIZERS=ON` 开启 ASan/UBSan（GCC/Clang），测试不依赖窗口或网络。

## 2026-09-12 本机验证

GCC 11.4 Debug 核心及 GRE 适配 9/9 通过；ASan/UBSan 9/9 通过。当前执行沙箱使 LeakSanitizer 报 ptrace 不支持，因此后者使用 `ASAN_OPTIONS=detect_leaks=0`；没有宣称 LSan 已通过。独立内存测试验证平台分配/释放计数平衡及失败回滚。

真实 GRE 离屏演示成功生成四张 480×360 图像，已检查球体位置随步进变化。6 秒弹球示例最终中心高度约 0.4995 m、竖直速度 0，与半径 0.5 m 和允许的 0.0005 m 穿透容差一致。

## 窗口集成

启用 GRE 窗口后增加 `ympme_gre_window`（共 10 组）。SDL dummy 驱动通过真实键盘事件验证渲染帧变化、暂停时步数不变、继续、重置、再次弹起和 Esc 退出。

## 三轮差速小车

新增 `ympme_car_drive` 和 `ympme_car_window`，GRE 窗口构建共 12 组测试。覆盖等速直行、反向原地旋转、恒曲率圆弧、万向轮前进/倒车回正、围栏和障碍阻挡、检查点、驾驶输入、刹车、暂停和双视角。

## 0.2.0 Mesh 与过滤

完整 GRE 窗口构建共 13 组。新增 `ympme_mesh_collision` 检查凸体拓扑/凸性验证、粗检测早退、AABB 假阳性、旋转 Mesh、Mesh/Mesh 与 Mesh/Sphere、包含深度、反序法线、双向组矩阵、同组排除、同装配体排除、原有 category/mask、Mesh 动态落地及步进零分配。小车测试增加零件碰撞体绑定、内部过滤早退及 H 精度切换。

## 三车悬挂测试场

完整 GRE 窗口构建共 15 组。`ympme_vehicle_suspension` 验证三角地形采样、PGM 往返、三类车静态载荷平衡、四轮 Ackermann 转向、左右独立悬挂压缩、姿态变化、离地/落地、三条完整山包路线和履带行程连续性。`ympme_vehicle_playground` 使用 SDL 事件切换并驾驶全部三辆车，检查同车过滤、异车检测、实际障碍阻挡，以及过山包的压缩差/倾角、帧变化、自动测试、暂停、重置、精度和相机切换。

新悬挂/高度场模型通过 GCC 的 `-Wall -Wextra -Wpedantic -Werror` 编译及 ASan/UBSan 检查（新模型源码启用检测，沿用已有核心静态库，`detect_leaks=0`）。

## 齿轮啮合演示

完整 GRE 窗口构建共 17 组。新增 `ympme_gear_model` 验证齿形采样、啮合相位、传动比、连续回绕、加速、反转和断电滑行；`ympme_gear_window` 检查 SDL 键盘事件、暂停/单步、调速反转、三视角、辅助图形、重置和渲染帧变化。

## 逐齿碰撞与回差

完整 GRE 窗口构建共 18 组。新增 `ympme_gear_contact`：两轴独立运动、分离不传动、从动轮反驱、无电机无增能、60 RPM 长期不跳齿、紧间距与加大间距的反向空程差异。当前测试测得输入轴空程约 0.31° / 3.33°，最大离散穿透约 0.621 mm。窗口测试改为三组同步对照，检查真实齿面接触和从动轴反转。新增模型通过 ASan/UBSan，`detect_leaks=0`。

## 飞行载荷与编译裁剪

默认 GRE 窗口构建增加 `ympme_features_core`，共 19 组；启用飞行扩展再增加 `ympme_flight`，共 20 组。前者检查只链接核心时不会继承飞行宏；后者根据实际裁剪项验证载荷运算、旋翼偏置/反扭矩、对称抵消、升阻力方向与平方速度、退化输入和失败回滚。飞行计算通过 ASan/UBSan，`detect_leaks=0`。

`python3 tests/check_features.py` 验证 5 个独立构建，包含 CTest、静态库成员/符号裁剪、核心库归档逐字节一致及 Body/World 布局一致；开发脚本依赖 Python 和 nm/ar，核心库不新增这些依赖。

## 皮带、滑轮组与绳端摆动

新增 `ympme_mechanisms_model` 和 `ympme_mechanisms_window`，默认完整 GRE 窗口构建共 21 组（另启用飞行扩展时为 22 组）。模型检查理想传动比、摩擦传动/反转、零摩擦不驱动从动轮、最高 UI 转速运行及断开摩擦后的减速、节点轮面排斥、有限伸长、4:1 绳长与提升行程、静态张力/释放/限位、质心从绳端零初速初始化、摆动范围和能量上界。窗口检查四幅渲染变化及暂停、单步、反转、摩擦、提升、相机与重置。

新模型源码经 `-Wall -Wextra -Wpedantic -Werror` 编译，并通过 ASan/UBSan（`ASAN_OPTIONS=detect_leaks=0`）。真实 SDL 桌面窗口也已执行键盘冒烟流程和布局检查。

## 独立 ROPE 模型与缓存 GRE 适配

启用 `YMPME_MODEL_ROPE` 的完整 GRE 窗口构建共 24 组；ROPE + 飞行同时启用时再增加飞行测试。新增 `ympme_rope`、`ympme_rope_gre`、`ympme_rope_window`，原机械模型/窗口测试验证摆绳迁移后行为。覆盖两端固定自然下垂、绳端挂物、松弛不推压、张力/柔度、移动和释放端点、闭合链、借用视图、非法输入不改状态、插值/缓存、重置复用与窗口操作。

本次 Release 全量 24/24 通过。关闭 Demo，仅启用 `YMPME_BUILD_ROPE_GRE` 和 ROPE 的独立适配构建 12/12 通过，不引入 SDL。GCC/Clang Linux 下包装 malloc/calloc/realloc，检查物理与适配循环分配次数为零；ROPE 源码及物理测试通过 ASan/UBSan，`detect_leaks=0`。

`tests/check_features.py` 扩为 7 组配置（原飞行五组、ROPE、ROPE+飞行），通过核心归档/ABI 一致性、功能宏隔离、关闭源文件/符号不存在、ROPE 无分配或渲染符号依赖检查。性能条件与实际测量见 `YMPME/MODEL/ROPE/PERFORMANCE.md`。

## 既有 Demo 复用 MODEL

启用全部八个模型但关闭 GRE / Demo 时为 19 组测试；启用全部 GRE 窗口 Demo 时为 30 组（均未开启飞行扩展）。齿轮、差速驱动和三车悬挂的原物理回归现可独立运行；地形 PGM 往返保留在 Demo 的 `ympme_terrain_io` 中。新增皮带、滑轮、地形与悬挂载荷的独立测试。

Release 无窗口 19/19、完整 GRE 30/30 通过；全部模型及核心使用 ASan/UBSan 构建后 19/19 通过，`ASAN_OPTIONS=detect_leaks=0`（没有声称 LSan 通过）。

`python3 tests/check_models.py` 验证十组配置：仅核心、七个叶模型、带必要依赖的 VEHICLE、全部模型。各配置运行 CTest，检查关闭模块的归档不存在、模型无 GRE/SDL/分配/文件 I/O 符号及可写全局状态、核心归档哈希和 ABI 一致；缺少 VEHICLE 依赖时配置明确失败。原 `check_features.py` 继续负责飞行和 ROPE 的七组裁剪组合。

正式 `build` 目录重新构建后 Debug 30/30 通过。通过 `gio launch` 实际启动原桌面入口：三轮小车、三车测试场、齿轮和机械演示；各运行 8 帧后自动退出，四份启动日志均为 `DEMO_EXIT=0`。

## 无人机与老鹰 AIRCRAFT

新增 `ympme_aircraft` 与 `ympme_flight_window`。物理测试覆盖无人机悬停、前进制动、风扰、转向、爬升、断电落地、非法输入回滚和实例独立；鹰的带动力飞行、无动力滑翔能量下降、翅膀展开、转弯四元数归一化，并运行 120 秒带风的振翅/滑翔交替回归。窗口测试覆盖两幅图像变化及真实 SDL 键盘事件操作。

仅启用 AIRCRAFT/FLIGHT 的无窗口构建 12/12 通过，ASan/UBSan 12/12 通过（`ASAN_OPTIONS=detect_leaks=0`）；独立 GRE 飞行场构建 15/15 通过。`check_features.py` 扩为八组配置，包含 AIRCRAFT：核心归档哈希/ABI 一致，关闭的模型源文件与符号不存在，AIRCRAFT 无渲染/分配符号，确实调用 FLIGHT 的旋翼和翼面接口；缺少任一飞行依赖时配置明确失败。

正式 `build` 目录启用全部模型和飞行扩展，Debug 全量 33/33 通过。通过 `gio launch` 验证新桌面入口，在真实 SDL 窗口运行 45 帧，启动日志 `flight_playground-487713.log` 显示 `DEMO_EXIT=0`，并检查了最终画面与完整按键提示。

## 控制反馈与 SENSOR

新增三类传感器测试及 `ympme_drone_control`：静止/自由落体比力、安装偏移转动项、坐标转换、噪声复现、采样时钟、编码器量化/跨圈、接触作用反作用、求解迭代冲量汇总、测力窗口/重复步拒绝、反馈溢出、普通/观测步进逐字节状态一致，以及无人机原始电机输入和带噪声/风扰的多速率闭环。

无窗口配置 16/16、独立 GRE 配置 19/19 通过。ASan/UBSan 无窗口 16/16 通过（`detect_leaks=0`），新增测力窗口及最终传感器修改另通过三项 sanitizer 回归。`check_sensors.py` 六组配置验证各模块独立裁剪、无渲染/堆分配依赖，核心归档/ABI 在功能开关之间一致。

正式 `build` 启用全部模型/传感器后 Debug **37/37** 通过；关闭三个传感器后的原内置飞行窗口冒烟亦通过。原桌面入口实际运行 30 帧并正常退出（`flight_playground-508986.log`，`DEMO_EXIT=0`），已检查真实 GRE 窗口的 IMU、载荷和编码器显示。

## 通用刚体 / 关节

`test_rigid` 验证三维惯量、施力点、陀螺项、极小角速度和反馈；`test_joint` 验证反作用、连续角、限位、电机、移动参考系与多节链；`test_model_pendulum` 验证周期和静态力矩；`test_joint_lab_control` 验证伺服/编码器累积/负载；`ympme_joint_window` 验证 GRE 交互。`python3 tests/check_joints.py` 检查 core/rigid/joint/pendulum/control 五种无渲染裁剪组合和 ABI 不变。

## 空间传感器

`test_query`验证Sphere/Box/旋转Mesh交点及过滤，`test_sensor_lidar`验证2D完整360°单圈与盲区，`test_sensor_depth`验证光轴深度/毫米量化/点云，`test_sensor_tactile`验证单元压力与载荷守恒；`test_sensor_scene`使用实际World检查重力接触与光学查询，`ympme_sensor_window`覆盖GRE交互。`python3 tests/check_spatial_sensors.py`验证六种独立裁剪配置及核心ABI/归档一致。

## 刚体 / 关节 / 接触与夹球

启用 DYNAMICS 新增 `ympme_dynamics`；启用 TACTILE 后增加 `ympme_grasp_physics`，GRE 窗口配置再增加 `ympme_grasp_window`。覆盖线/角动量、偏心碰撞旋转、摩擦滚动、多点落地、过滤/失败回滚、电机与接触抵消、抓取提升和释放、零摩擦失败、双侧力图归集，以及 GRE 按键/渲染集成。细节见 [实验说明](../project_Demo/grasp_lab/README.md)。

## 凸体抓取与按需感知

新增 ympme_convex_dynamics，覆盖旋转盒子/Mesh 各组合的稳定承重、球/Mesh、Mesh/平面、偏移包围盒、失败与过滤，以及 600 组随机旋转与 SAT 一致性。grasp_lab 三种对象分别验证提升、释放、零摩擦失败；只有两指带触觉，物体直接读求解载荷。窗口回归切换球、方块和凸体并检查提升。
