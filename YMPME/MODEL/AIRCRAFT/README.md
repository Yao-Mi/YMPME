# AIRCRAFT：无人机与老鹰飞行预设

`YMPME::aircraft` 是独立、无窗口、无堆分配的游戏飞行模型，显式复用 `YMPME::flight` 的旋翼和翼面载荷计算。`YMPME_MODEL_AIRCRAFT` 默认 OFF；开启时须同时开启 `YMPME_ENABLE_FLIGHT`、`YMPME_FLIGHT_ROTORS`、`YMPME_FLIGHT_AERODYNAMICS`。关闭模型不会改变 CORE 的 ABI，也不引入 GRE / SDL。

- 无人机：四个旋翼的平方推力、反扭矩、电机响应滞后；速度/垂直速度控制、姿态稳定、四旋翼混控。加速时倾斜机体，外界风通过相对速度阻力影响飞行；关闭电机后失去支承。
- 老鹰：由来流和攻角计算等效升力系数、阻力及简化失速衰减；周期振翅产生前向推力，停翅保留被动升阻力；转弯滚转和低速压低机头由简化稳定控制器完成。翅膀振幅与振翅功率同步。
- 两种预设独立积分位置、速度、机体角速度及单位四元数。姿态控制将 Y-X-Z 欧拉角速率转换到机体坐标，包含陀螺项补偿。调用方不可再用 World 对同一状态重复积分。

```c
#include "MODEL/AIRCRAFT/YMPME_Aircraft.h"
pme_aircraft drone;
pme_aircraft_command command = {0, 0, 0, 0, 1};
pme_result result = YMPME_Aircraft_Init(&drone, PME_AIRCRAFT_DRONE,
                                      YMPME_Vec3(0, 5, 0));
if (result == PME_OK)
    result = YMPME_Aircraft_Step(&drone, &command, YMPME_Vec3(0, 0, 0), 1.0f/240);
/* 读取 drone.position / q / rotorPhase；显示端按需创建网格和更新顶点。 */
```

单位为米、千克、秒、弧度；惯量 kg·m²、力 N、力矩 N·m。世界 Y 向上，局部 Z 向前、X 向右。初始化需要中心高度至少 0.3 m；无人机静止、老鹰初速 10 m/s。状态和缓存随实例存储；步进接受 `0 < dt <= 1/120 s`，建议 1/240 s。命令各轴为 [-1,1]，功率为 [0,1]。输入无效或计算失败时不改实例。显示线程与步进线程不可无保护地并发读写。

这是受控飞行预设，不是高保真航空/鸟类仿真。飞机参数目前沿用示例固定值；鸟翼是单个等效翼面，翅膀几何不逐片参加空气计算。`power=0` 时鸟的被动稳定控制仍工作，无前向振翅推力；无人机旋翼按电机时间常数减速。只有 y=0 平面的简化落地支承，不接入机体互撞或树木碰撞。鹰落地后用 Demo 的 Home 重新从空中开始，不包含起跑/扑翼起飞控制。

主机读取连续状态，GRE Demo 的网格、局部顶点和法线在初始化时缓存，常规帧更新不重建网格。自动路线和相机属于 Demo，不写入物理模块。

## 真值读取与直接执行器输入

`YMPME_Aircraft_GetFeedback` 提供统一状态、世界角运动和完整物理步的平均加速度；初始导数无效。`YMPME_Aircraft_StepMotors` 仅适用无人机，接受四个 [0,1] 转速比例，跳过内置控制器/混控，保留旋翼与电机滞后。模型仍不依赖传感器或应用控制器。新版本实例增加导数缓存，应重新编译使用方。
