# 可复用物理模型层

本目录用于存放可独立使用、可按编译选项裁剪的物理模型。ROPE、皮带、滑轮、齿轮、差速驱动、地形、悬挂和车辆组合模型已被原 Demo 直接复用。

## 分层与依赖

- `CORE`：通用积分、几何接触与约束求解能力。
- `MODEL`：组织物理节点、连接关系、机构参数、状态和载荷，复用核心能力。
- `Demo` / `project_Demo`：创建模型、处理输入、布置场景及 GRE 显示。

依赖方向为 **Demo → MODEL → 核心库**。模型不得包含 GRE、YMGUI、SDL 类型，不持有渲染网格、相机或窗口。渲染层读取模型状态，通过应用绑定转换坐标和尺度。

## 已复用的模型

| 模块 / 目标 | 使用它的 Demo |
| --- | --- |
| ROPE / `YMPME::rope` | 绳索实验、机械演示的摆绳 |
| BELT / `YMPME::belt` | 机械演示的理想 / 摩擦皮带 |
| PULLEY / `YMPME::pulley` | 机械演示的 4:1 提升机构 |
| GEAR / `YMPME::gear` | 理想、紧耦合和回差齿轮 |
| DRIVE / `YMPME::drive` | 三轮小车和三车测试场，共用一份实现 |
| TERRAIN / `YMPME::terrain` | 三车测试场的高度数据与采样 |
| SUSPENSION / `YMPME::suspension` | 三车模型的支承载荷 |
| VEHICLE / `YMPME::vehicle` | 三车测试场，组合 DRIVE / TERRAIN / SUSPENSION |

每个模块的 README 和头文件说明当前预设范围。原 Demo 的输入、相机、GRE 网格、障碍规则、检查点及 PGM 文件读写仍在应用层。自由落体 / 弹球直接复用 CORE 的 World/Body，无需再包一层只有转发功能的 Model。已有 EXT/FLIGHT 保持独立可选扩展。

## 编译和调用

每个 `YMPME_MODEL_<名称>` 开关默认 OFF，开启后显式链接对应目标。VEHICLE 必须同时开启 DRIVE、TERRAIN、SUSPENSION。构建相关 Demo 时需显式启用所需模型；CMake 按模型是否启用决定是否构建对应 Demo。

例如只用差速驱动：

```cmake
set(YMPME_MODEL_DRIVE ON CACHE BOOL "" FORCE)
add_subdirectory(path/to/YMPME ympme-build)
target_link_libraries(my_game PRIVATE YMPME::drive)
```

```c
#include "MODEL/DRIVE/YMPME_Drive.h"
pme_drive car;
YMPME_Drive_Reset(&car);
/* 固定时钟中推进；显示层可直接读取 car.x / car.z / car.yaw。 */
pme_result result = YMPME_Drive_Step(&car, 1.0f, 1.0f, 1.0f/120);
```

用皮带时初始化 `pme_belt`，调用 `YMPME_Belt_Init` / `YMPME_Belt_Step`；用滑轮组则初始化 `pme_pulley`，调用 `YMPME_Pulley_Init` / `YMPME_Pulley_Step`。这些调用直接进入库实现，Demo 不再保存另一份求解器。

## 接口与内存

沿用项目 C99、`YMPME_` 函数前缀、`pme_` 值类型和 `PME_` 指针类型。各模型按需要提供描述参数、状态、初始化、输入 / 端点更新、步进和状态读取；明确由调用方还是 World 推进，避免重复积分。

公开接口必须说明米 / 千克 / 秒等单位、参数范围、坐标系、缓存容量与所有权、失败行为，以及理想模型或近似模型的边界。外部缓冲优先；步进期间不动态分配。可选 Create / Free 经平台内存接口分配释放。支持多个独立实例，不依赖全局可变状态。

## 裁剪与验证

每个模型随实现增加独立 CMake 目标及对应开关；未启用时不编译其源文件。只使用核心时不链接模型、不继承模型功能宏，也不改变核心结构体布局。存在模型间依赖时必须在 CMake 中明确声明和校验。

实例的精度 / 传动方式选项与编译裁剪分开。默认核心构建不因示例引用而隐式引入全部模型。当前八个模型均可单独裁剪；ROPE 的 GRE 适配另由 `YMPME::rope_gre` 提供。十组合裁剪检查同时验证核心归档/ABI 不变、模型中没有渲染/分配/I/O符号和可写全局数据。

每个模型需要可判定的无窗口测试，再由 Demo 验证显示与交互。绳索首先覆盖端点固定、自然下垂、长度误差、载荷与能量、初始挂物位置及多实例独立性；迁移既有模型时保留原有回归，并验证关闭模块的构建。

## 高效对接约定

模型独立于 GRE，同时为高效读取保留明确的数据通道。可选 BRIDGE / 应用绑定依赖模型和渲染器，直接读连续状态数组；模型不存储 GRE 类型，也不调用 GRE 函数。

- 提供只读借用视图，说明数量、有效期和线程约束；读取不生成整份状态副本。
- 初始化 / 拓扑变更时建立绑定与网格，常规帧复用内存、索引和局部几何。
- 每个显示帧按需同步，物理子步不重复生成渲染数据；支持区分状态与拓扑变更。
- 适配层一次遍历完成必要的插值、单位转换和顶点更新；布局不同的转换仍需写入 GRE 缓冲，不把“独立”包装成未经验证的零拷贝承诺。
- 同步与模型步进不得无保护地并发；多线程读取采用明确的同步或快照缓冲策略。
- 分别记录模型、同步和绘制耗时，并与同条件直接绑定比较；正常帧步进与适配同步无堆分配。

ROPE 已实现上述数据视图和缓存对接，测量条件及结果见 [性能记录](ROPE/PERFORMANCE.md)。

## AIRCRAFT 飞行预设

新增第九个模型 [AIRCRAFT](AIRCRAFT/README.md)，由无人机/老鹰 Demo 直接复用。开关 `YMPME_MODEL_AIRCRAFT` 默认 OFF，显式依赖 EXT/FLIGHT 及旋翼/气动两个子模块；不依赖 GRE。原八个模型的十组矩阵仍由 `check_models.py` 验证，AIRCRAFT 加入 `check_features.py`，该脚本现为八组配置。

可选 SENSOR 层与 MODEL 平级，消费统一真值，不反向引入模型或渲染依赖；详情见 [传感器与反馈](../SENSOR/README.md)。

## 通用关节组装

新增 [PENDULUM](PENDULUM/README.md) 单摆 Model，依赖独立 [JOINT](../JOINT/README.md) 模块。关节约束位于 JOINT，惯量和通用转动位于 CORE/RIGID，模型只负责组装；控制器和传感器在应用侧按需接入。现有齿轮、皮带、滑轮等专用算法保持现状，不强制改为通用刚体求解。

`SURFACE_ATLAS`：纯 C99 等面积三角形传感器布局，默认 `YMPME_MODEL_SURFACE_ATLAS=OFF`；启用后链接 `YMPME::surface_atlas`。优先保留拓扑相邻边，不修改真实面积或蒙皮，仅生成显示/拾取用 UV，不依赖 GRE/SKE。详见 [说明](SURFACE_ATLAS/README.md)。
