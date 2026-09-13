# YMPME — mini 物理运动引擎

YMPME 是面向 PC / MCU 和小游戏的 C99 三维运动物理库，与 YMGRE 渲染库配合使用。当前版本 **0.2.0**，提供可运行的基础运动、碰撞及 GRE 对接示例。

## 已实现

- Static / Dynamic / Kinematic 三类物体；重力、力、冲量、线性阻尼。
- 球–球、球–轴对齐盒、盒–盒碰撞；包含球心在盒内的分离处理。
- 迭代法向冲量、恢复系数、库仑摩擦、穿透修正和低速反弹阈值。
- 过滤 → AABB → 可选凸 Mesh SAT 精检测；支持旋转网格及 Mesh/Box/Sphere。
- 碰撞类别/掩码、32 组配对矩阵、同装配体内部忽略、Sensor 查询。
- 固定时间步累积、最大追赶子步、丢弃时间统计和渲染位置插值。
- 固定容量物体槽；外部静态缓存 Init / Destroy、PC Create / Free。
- 可选 GRE 适配：从不可变局部网格重建世界顶点，避免变换累加。

默认物理单位是 **米、千克、秒**，Y 向上，重力 `(0,-9.81,0)`；默认旧 World 不积分角速度；可选 RIGID/JOINT 提供通用转动和机构约束。旧 World 的 Mesh 姿态由应用设置，并保持与 GRE 的轴向一致。

## 人体皮肤传感器

新增独立 **人体皮肤传感器** demo：左手摸胸腹与球形探头两种模式，共用一张 64×64 全身拓扑网格图（面片各占一格，V 切回 512×512 三角图）。新增 **人体地面运动** demo，可观察行走、站立、下蹲和离地时的双脚接触力，并提供固定全身 RGB、面片 ID 与浮点力观测，便于接智能体。两者使用完整六槽蒙皮，构建统一位于 `build/human`，可执行文件位于 `build/project_Demo/human_skin` 和 `build/project_Demo/human_ground`。参见 [人体皮肤](project_Demo/human_skin/README.md) 与 [地面运动](project_Demo/human_ground/README.md)。

## 把玩球与三维皮肤力反馈

新增独立 **把玩球** demo：低模手抓握 128 面球、手掌移动/倾斜、松手、拨球和自旋；球具有独立平移/转动动力学，提供线框切换。默认使用 GRE 自动展开的 UV 显示 RGB 三分量力图（R 法向、G/B 两切向，128 表示零切向力），可切换力模长、压力图及稀疏均匀方向箭头。运行 `./build/project_Demo/hand_ball/demo_hand_ball`；操作、近似接触边界和可裁剪构建见 [把玩球](project_Demo/hand_ball/README.md)。原皮肤按压 demo 保留。

## 传感器实验台

新增独立可裁剪的 **2D 360°激光雷达、深度相机与触觉阵列基础模块**。雷达默认360束/1°，直接查询真实物理几何；深度输出光轴Z距离，可转uint16毫米图；触觉消费实际接触载荷。三者与原IMU、编码器、六维力传感器并列。

运行 `./build/project_Demo/sensor_lab/demo_sensor_lab`。WASD移动探头、Q/E转向、R切雷达量程、F加载重物。详见 [实验台操作](project_Demo/sensor_lab/README.md)、[传感器模块](YMPME/SENSOR/README.md)。当前触觉接触位置仍为AABB估计，不是精确皮肤压力分布；射线查询暂未加入BVH。

## 关节手指夹球

新增可裁剪的 `YMPME::dynamics`，将旋转刚体、关节与球/旋转盒子/凸 Mesh/平面接触放在同一子步交替求解，输出两侧接触力和力矩。开启 `YMPME_ENABLE_DYNAMICS=ON`；物理层不依赖 GRE/SKE/SENSOR。

运行 `build/project_Demo/grasp_lab/demo_grasp_lab`，观察推球滚动，以及球、方块、削角凸体的抓取对照（2/3/4 切换）。只有两指带触觉，物体直接显示接触载荷。`F` 关闭摩擦可以对照抓取失败；`T` 切手动，`A/D` 闭合/张开，方向键升降。详见 [夹球实验](project_Demo/grasp_lab/README.md) 和 [共同求解 API / 范围](YMPME/DYNAMICS/README.md)。支持球、旋转盒子、凸 Mesh 相互接触及与静态平面接触；尚无 CCD，也未自动迁移旧 demo。

## 通用刚体与关节

新增独立的 **RIGID → JOINT → PENDULUM Model**：惯量/四元数动力学、固定/转动/滑动关节、速度或力矩电机、限位、约束反作用力与完整运动反馈。均可裁剪，无 GRE/SENSOR 依赖，旧 Body/World ABI 保持不变。

运行 `./build/project_Demo/joint_lab/demo_joint_lab`，在 GRE 窗口对照电机单摆、固定悬臂与滑动平台。操作、uint16 编码器反馈见 [实验台](project_Demo/joint_lab/README.md)，API 见 [JOINT](YMPME/JOINT/README.md)。**当前通用旋转刚体尚未接入旧 World 的形状碰撞求解**，不要对同一物体重复调用两个积分器。

## 可裁剪扩展

飞行相关功能是独立、默认关闭的扩展：`YMPME_ENABLE_FLIGHT=ON` 才构建 `YMPME::flight`，旋翼和气动力可再用 `YMPME_FLIGHT_ROTORS`、`YMPME_FLIGHT_AERODYNAMICS` 分别裁剪。关闭的源文件不编译、对应接口不暴露，核心 Body/World 不增加字段。

当前飞行扩展提供旋翼推力/反扭矩和升阻力**载荷计算**，尚不含完整旋转动力学及飞控。构建、MCU 手工裁剪和接口说明见 [飞行扩展](YMPME/EXT/FLIGHT/README.md)。GRE 适配、窗口、Demo 和测试仍使用各自原有开关，普通游戏继续只链接核心。

## 文件分布

沿用 YMGRE 的模块分布、命名及 Demo 分层，新增独立适配层：

```text
YMPME/
├── CONFIG/       公共定义、类型、可替换内存接口
├── CORE/         向量、碰撞、求解器、世界与时间步；可选 RIGID
├── JOINT/        可选通用固定/转动/滑动关节
├── DYNAMICS/     可选旋转刚体/关节/接触共同求解
├── MODEL/        可复用机构与运动模型
├── SENSOR/       可裁剪 IMU、编码器、力传感器
├── EXT/          可裁剪载荷扩展
├── OPOBJ/        物体描述、初始化、创建与释放
├── DEBUG/        平移动能等调试统计
├── IOFILE/       文件格式扩展边界说明
├── BRIDGE/       可选 GRE 适配
└── YMPME.h       聚合头文件
Demo/            单模块最小示例
project_Demo/    持续游戏时钟及 GRE 渲染示例
Resource/        示例资源约定
extern_lib/      外部依赖约定（不复制 GRE 源码）
tests/           无窗口 CTest 回归测试
```

## 直接运行 Demo

项目不安装桌面或应用菜单入口，也不提供启动包装脚本。构建完成后直接运行可执行文件；以下路径相对于 YMPME 根目录。数值示例在终端运行以查看输出。

| Demo | 可执行文件 |
| --- | --- |
| 人体皮肤传感器 | `build/project_Demo/human_skin/demo_human_skin` |
| 人体地面运动 | `build/project_Demo/human_ground/demo_human_ground` |
| 把玩球 · 三维触觉 | `build/project_Demo/hand_ball/demo_hand_ball` |
| 低模手皮肤反馈 | `build/project_Demo/hand_skin/demo_hand_skin` |
| 传感器实验台 | `build/project_Demo/sensor_lab/demo_sensor_lab` |
| 关节手指夹球 | `build/project_Demo/grasp_lab/demo_grasp_lab` |
| 关节控制实验台 | `build/project_Demo/joint_lab/demo_joint_lab` |
| 无人机与老鹰飞行场 | `build/project_Demo/flight_playground/demo_flight_playground` |
| 猫头鹰键盘飞行 | `build/project_Demo/owl_flight/demo_owl_flight` |
| 自由落体 | `build/demo_free_fall` |
| 弹球数值演示 | `build/project_Demo/bouncing_ball/demo_bouncing_ball` |
| GRE 弹球窗口 | `build/project_Demo/gre_bouncing_ball/demo_gre_bouncing_ball` |
| 三轮差速小车 | `build/project_Demo/three_wheel_car/demo_three_wheel_car` |
| 三车悬挂测试场 | `build/project_Demo/vehicle_playground/demo_vehicle_playground` |
| 通用绳索模型 | `build/project_Demo/rope_lab/demo_rope_lab` |
| 皮带滑轮与绳摆 | `build/project_Demo/mechanisms/demo_mechanisms` |
| 齿轮约束与碰撞对照 | `build/project_Demo/gear_mesh/demo_gear_mesh` |

所有产物统一放在 `build` 中：主工程直接使用 `build`，六槽人体使用 `build/human`，四槽猫头鹰使用 `build/owl_flight`；窗口可执行文件统一在 `build/project_Demo/<demo>/`。修改源码后使用对应 CMake 构建目录重新编译。

## 构建与运行

仅需 C99 编译器、CMake 3.16+；默认不依赖 GRE、YMGUI 或 SDL。

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build -j
ctest --test-dir build --output-on-failure
./build/demo_free_fall
./build/project_Demo/bouncing_ball/demo_bouncing_ball
```

嵌入其他 CMake 项目：

```cmake
add_subdirectory(path/to/YMPME ympme-build)
target_link_libraries(my_game PRIVATE YMPME::ympme)
```

MCU 构建关闭 `YMPME_BUILD_DEMOS`、`YMPME_BUILD_TESTS`，使用工具链文件。`YMPME_World_Init` 可绑定静态数组；仅需静态内存时，不必调用 Create / Free。浮点实现使用 `libm`；实际 MCU 性能尚未测量。

## 齿轮啮合演示

运行 `./build/project_Demo/gear_mesh/demo_gear_mesh`，同屏比较 **理想约束、紧间距逐齿碰撞、加大间距回差碰撞**。碰撞组的两轴独立积分，由实际齿面接触冲量传递扭矩；中心距分别为 3.00 / 3.08 m。

**T 开启低速自动反转，C 切换啮合近景，Space 暂停、N 单步**。↑/↓ 调速，R 手动反转，M 电机/惯性滑行。状态栏显示实际两轴 RPM、接触扭矩与相位误差，红点标出接触位置。

当前为固定轴平面齿形碰撞，模型边界、操作和回差测试见 [齿轮说明](project_Demo/gear_mesh/README.md)。

## 三车与悬挂测试场

运行 `./build/project_Demo/vehicle_playground/demo_vehicle_playground`：同场驾驶履带车、三轮差速车和带四轮独立弹簧阻尼悬挂的小车。地面有实际几何山包，支持加载灰度高度图；渲染和轮地接触共用地形三角面。

**1/2/3 切车，WASD 驾驶，T 自动过山包，C 切换跟随/俯视/悬挂侧视。** 状态栏实时显示四角压缩量、接地数和车身姿态。H 切换 Mesh/AABB，Space 刹车，P 暂停，R 重置。

模型、操作和高度图格式见 [测试场说明](project_Demo/vehicle_playground/README.md)。车辆示例单独积分简化悬挂，物理核心仍保持现有的平移刚体接口。

## 三轮差速小车

运行 `./build/project_Demo/three_wheel_car/demo_three_wheel_car`，在 GRE 窗口驾驶两差速轮加后万向轮的小车。W/S 前后、A/D 转向、Space 刹车、左 Shift 加速、C 切换视角，H 切换 Mesh/AABB（回到起点）；收集五个绿色检查点。模型及边界见 [小车说明](project_Demo/three_wheel_car/README.md)。

## GRE 实时窗口

直接运行 `./build/project_Demo/gre_bouncing_ball/demo_gre_bouncing_ball`，打开 640×500 的 GRE/YMGUI/SDL 弹球窗口（需要 SDL2 开发包及桌面会话）。

- 空格：暂停/继续；R：从 4 m 高处重新释放。
- ↑：以 6 m/s 再次弹起；Esc 或关闭窗口：退出。
- 底部实时显示高度、竖直速度和模拟时间。

## GRE 联动构建

```sh
cmake -S . -B build \
  -DYMPME_BUILD_GRE_DEMO=ON \
  -DYMPME_GRE_ROOT=/home/yaomi/文档/0502_YMGRE
cmake --build build -j
ctest --test-dir build --output-on-failure
cd build
./project_Demo/gre_bouncing_ball/demo_gre_bouncing_ball
```

默认打开实时窗口。需要离屏图像时加 `--headless`，输出 `ympme_ball_000.ppm`、`060.ppm`、`120.ppm`、`240.ppm` 四张真实 GRE 图像。无 SDL 的机器可配置 `-DYMPME_GRE_WINDOW=OFF` 保留纯离屏构建；只做位置同步时可单独打开 `YMPME_BUILD_GRE_BRIDGE=ON`，只依赖 GRE 头文件。

渲染单位换算示例：`unitsPerMeter=20` 表示物理 1 m 对应 GRE 20 单位；半径 0.5 m 的物理球对应 GRE 半径 10 的网格。模型缩放不自动改变碰撞形状。

## 当前边界

这是小规模场景的基础版本。盒体始终轴对齐，球只做平移；支持外部设置凸 Mesh 的旋转姿态，但尚无旋转刚体、角动量、独立 OBB 类型、凹网格/三角汤、关节、休眠或连续碰撞检测。高速小物体可能穿透，应减小时间步或加厚碰撞体；不承诺复杂堆叠稳定性。碰撞遍历复杂度为 `O(迭代数 × 容量²)`，建议从几十个物体开始测量。

数值输入应保持小游戏常用尺度，避免接近浮点极限；浮点运算不保证不同 CPU / 编译器之间逐位确定性。相同环境、相同输入和子步序列可复现结果。

接口及所有权见 [API.md](API.md)，分层和后续路线见 [ARCHITECTURE.md](ARCHITECTURE.md)、[PROJECT.md](PROJECT.md)。

Mesh 数据、精度开关、碰撞组和所有权示例见 [MESH_COLLISION.md](MESH_COLLISION.md)。

## GRE 窗口与输入边界

PME 的物理库和全部 demo 源码均不直接包含或调用 SDL。窗口、平台输入采集、按键状态、计时、标题和显示兼容设置统一由 GRE `demo_host.h` 提供。应用通过 `BindKeys` 声明操作，再读取 `HeldKeys` / `TakeActions`；测试通过 GRE `InjectKey` 和 `InjectFocusLost` 注入输入。原有 YMGUI 控件回调仍由 GRE 窗口宿主驱动。

桌面后端内部仍使用 SDL 软件显示；GLX 兼容默认设置由 GRE 初始化，程序可直接双击运行。无窗口测试使用 `YMGRE_HEADLESS=1`。边界测试 `ympme_demo_host_boundary` 防止平台 API 再次进入 PME 源码。

## 皮带、滑轮组与绳端摆动

运行 `./build/project_Demo/mechanisms/demo_mechanisms`。同一个 GRE 窗口包含约束式皮带、接触摩擦式皮带、4:1 滑轮组提升 8 kg 重物，以及从倾斜绳端零初速释放的 2 kg 悬挂物。

默认自动演示。**W/S 提升/下降，F 切换碰撞皮带摩擦，T 切换自动演示，Space 暂停、N 单步**。R 反转，↑/↓ 调速，H 松开/保持提升手柄，K 推动摆锤，Home 重置。模型及操作详见 [机械演示说明](project_Demo/mechanisms/README.md)。模型放在示例目录，关闭 GRE 示例构建即可一并裁掉，不增加核心或飞行模块依赖。

## 独立绳索模型

`MODEL/ROPE` 已支持两端固定自然下垂、绳端挂物、三维运动锚点、释放 / 固定节点和闭合链。模型只依赖物理核心，外部缓冲、连续状态视图、步进零堆分配；可选 GRE 适配直接读取节点并复用网格。

运行 `./build/project_Demo/rope_lab/demo_rope_lab`。Space 暂停、K 推动、T 锚点运动、P 释放末端、C 视角、Home 重置。窗口同时显示物理、同步和绘制耗时。

库使用 `-DYMPME_MODEL_ROPE=ON` 和 `YMPME::rope`；默认关闭。独立适配使用 `YMPME_BUILD_ROPE_GRE=ON` 和 `YMPME::rope_gre`，不要求 Demo 或 SDL。构建机械演示和绳索演示时需传入 `-DYMPME_MODEL_ROPE=ON`。说明与示例见 [ROPE](YMPME/MODEL/ROPE/README.md)。

## 既有 Demo 的模型复用

齿轮 → GEAR；三轮小车 → DRIVE；三车测试场 → VEHICLE + DRIVE/TERRAIN/SUSPENSION；机械演示 → BELT/PULLEY/ROPE。物理代码在 MODEL 中只有一份，Demo 保留输入、显示和场景规则。自由落体与弹球继续直接使用核心物理。

这些模型通过 `YMPME_MODEL_<名称>` 独立裁剪，默认关闭。若手工构建全部窗口 Demo，除 ROPE 外还应开启 BELT、PULLEY、GEAR、DRIVE、TERRAIN、SUSPENSION、VEHICLE。依赖和调用示例见 [模型总览](YMPME/MODEL/README.md)。

## 无人机与老鹰

新增双视角 GRE 飞行场，支持无人机悬停/操控、老鹰振翅/滑翔、阵风、自动飞行和三种视角。运行 `./build/project_Demo/flight_playground/demo_flight_playground`。物理复用默认可裁剪的 `YMPME::aircraft` 与已有 `YMPME::flight`。见 [操作说明](project_Demo/flight_playground/README.md)。

## 控制仿真与传感器

新增可选 IMU、编码器、接触测力模块，以及统一物理反馈、直接电机指令。飞行 Demo 默认运行“240 Hz 采样 → 120 Hz 独立控制器 → 四电机”的闭环，B 可切回内置控制；左侧显示传感器读数。构建时启用 IMU、ENCODER、FORCE 传感器模块。能力与精度边界见 [SENSOR 说明](YMPME/SENSOR/README.md)。

## 低模手皮肤反馈

新增[hand_low皮肤反馈实验台](project_Demo/hand_skin/README.md)，复用独立YMSKE低模手与15个PME关节，提供球形压头、曲面压力/剪切力、反作用测力和GRE热图。`YMPME_MODEL_SKIN`和`YMPME_BUILD_HAND_SKIN_DEMO`默认OFF，核心不引入SKE/GRE；该版是采样柔顺接触近似，尚非完整抓取/软组织仿真。

## 独立预编译 SDK

默认交付包位于 `build/YMPME_libs`，只有一份包含全部 PME 功能的 `libympme.a`，配套头文件默认开启全部接口。外部工程 `find_package(YMPME CONFIG REQUIRED)` 后链接 `YMPME::ympme` 即可。全部功能可用不会自动为物体创建传感器或 skin。

在源码根目录运行 `python3 sdk/build.py` 可重新构建、搬移验证并发布一体化 SDK。原分模块流程保留为 `python3 sdk/build_modular.py`，输出 `build/YMPME_libs_modular`；仅核心包使用 `python3 sdk/build_modular.py --profile core --output build/YMPME_core_libs`。两个流程使用独立构建目录，不更改现有 demo 配置。

详见 [一体化 SDK 说明](sdk/README-unified.md)、[分模块 SDK 说明](sdk/README.md) 和 [使用手册](sdk/手册.md)。
