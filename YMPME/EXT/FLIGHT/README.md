# 可裁剪飞行载荷扩展

飞行扩展独立于核心，默认不构建。**当前提供载荷计算接口，不是完整飞行动力学或飞控。** 通用角速度/转动惯量积分、姿态控制器、翼型/失速和旋翼入流模型仍待实现。

## 编译裁剪

| CMake 选项 | 默认值 | 作用 |
| --- | --- | --- |
| `YMPME_ENABLE_FLIGHT` | OFF | 是否创建独立的 `ympme_flight` / `YMPME::flight` 静态库目标 |
| `YMPME_FLIGHT_ROTORS` | ON | 主开关启用时，是否编译旋翼载荷源文件和暴露接口 |
| `YMPME_FLIGHT_AERODYNAMICS` | ON | 主开关启用时，是否编译气动力源文件和暴露接口 |

主开关 OFF 时，子选项即使为 ON 也不会构建任何飞行代码。子选项全关时，只保留力/力矩载荷的基础组合运算。现有车辆、齿轮和弹球只链接 `YMPME::ympme`，启用飞行目标也不会让它们自动获得飞行依赖。

示例：只要旋翼，不要气动力：

```sh
cmake -S . -B build-flight \
  -DYMPME_ENABLE_FLIGHT=ON \
  -DYMPME_FLIGHT_ROTORS=ON \
  -DYMPME_FLIGHT_AERODYNAMICS=OFF
cmake --build build-flight -j
ctest --test-dir build-flight --output-on-failure
```

应用按需链接：

```cmake
set(YMPME_ENABLE_FLIGHT ON CACHE BOOL "" FORCE)
set(YMPME_FLIGHT_ROTORS ON CACHE BOOL "" FORCE)
set(YMPME_FLIGHT_AERODYNAMICS OFF CACHE BOOL "" FORCE)
add_subdirectory(path/to/YMPME ympme-build)
target_link_libraries(my_flight_game PRIVATE YMPME::flight)
# 普通应用仍只链接 YMPME::ympme。
```

`YMPME::flight` 将对应的 `PME_ENABLE_FLIGHT`、`PME_FLIGHT_ROTORS`、`PME_FLIGHT_AERODYNAMICS` 宏传给消费者，`YMPME.h` 自动暴露所选接口。只链接核心的消费者三个宏仍为 0。没有全局生成的开关头，因此同一构建里可以同时存在核心消费者和飞行消费者。

## MCU / 不使用 CMake

基础核心源文件保持原样。开启飞行时统一定义 `PME_ENABLE_FLIGHT=1` 并加入 `YMPME_Flight.c`；需要旋翼时再定义 `PME_FLIGHT_ROTORS=1` 并加入 `YMPME_Rotor.c`；需要气动力时再定义 `PME_FLIGHT_AERODYNAMICS=1` 并加入 `YMPME_Aerodynamics.c`。未使用的源文件直接不参与编译，未定义的宏默认为 0；不能在关闭主模块时单独打开子模块。

模块只使用标准数学函数和值类型，无堆分配、GRE、SDL、文件 I/O 或线程依赖。接口返回参数错误时不改写输出。现有 `pme_body` / `pme_world` 没有新增字段；旋翼个数和载荷缓存由调用方决定。

## 现有载荷模型

- `YMPME_Flight_LoadAtPoint`：计算给定力关于质心的力矩 `r × F + freeTorque`。
- `YMPME_Flight_AddLoads`：合并多个载荷，支持输出与输入别名。
- `YMPME_Flight_RotorLoad`：简化静态平方律 `T = kT ω²`、机身反扭矩 `Q = -kQ ω|ω|`，再计入旋翼偏置产生的力矩。系数由应用提供。反转改变反扭矩符号；推力轴保持由 `axis` 指定，这不是可逆桨气动模型。
- `YMPME_Flight_AeroLoad`：用应用提供的面积、Cl、Cd 计算升阻力。阻力与物体相对空气速度反向；升力轴投影到垂直速度的平面。升力方向退化且请求非零升力时返回参数错误。零速度、零密度、零面积返回零载荷。

气动力采用 `q = ρ|v|²/2`、`L = q S Cl`、`D = q S Cd`；公式参考 [NASA 升力方程](https://www1.grc.nasa.gov/beginners-guide-to-aeronautics/lift-equation/) 和 [NASA 阻力方程](https://www1.grc.nasa.gov/beginners-guide-to-aeronautics/drag-equation/)。Cl/Cd 不在模块内根据攻角或马赫数自动生成。

输入及输出使用同一坐标系；位置相对质心，单位 m，速度 m/s，角速度 rad/s，力 N，力矩 N·m。应用若在机体系计算，需用姿态分别旋转力和力矩到目标坐标系。

```c
#include "YMPME.h"  /* 该目标应链接 YMPME::flight 并启用旋翼。 */
pme_rotor_desc rotor = {{0.3f, 0, 0}, {0, 1, 0}, 1e-5f, 1e-7f};
pme_flight_load load;
if (YMPME_Flight_RotorLoad(&rotor, 500.0f, &load) == PME_OK) {
    /* load.force 与 load.torque 都是载荷结果，尚未推进任何刚体。 */
}
```

模块不会自动调用 `World_Step`、添加重力或把力矩悄悄忽略。现有核心只能接收平移力；完整飞行运动需要后续通用旋转动力学或应用自己的角运动求解器接收 `load.torque`。

## 裁剪验证

`python3 tests/check_features.py` 在临时目录构建 5 种组合：核心、仅飞行基础、仅旋翼、仅气动力、全部启用。每种运行对应 CTest，检查静态库的成员和公开符号，确认关闭项未进入库；核心消费者不继承飞行宏。当前 GCC/Linux Release 验证中，五种组合的 `libympme.a` 逐字节一致，Body/World 大小一致。

单独开发检查脚本使用 Python、CMake、nm/ar；这些不是库的运行依赖。用 `--work-dir 新目录` 可保留构建与日志。`test_flight` 另验证偏置力矩、对称旋翼抵消、平方速度关系、阻力耗能方向、升力正交性、真空/静止和失败不修改输出。已通过 ASan/UBSan（未启用 LeakSanitizer）。

## AIRCRAFT 消费者

`MODEL/AIRCRAFT` 复用本扩展的旋翼和气动载荷，独立维护预设飞行器的平动及转动状态。FLIGHT 自身仍只计算载荷，不修改 World 或引入 Demo/GRE。详见 [模型说明](../../MODEL/AIRCRAFT/README.md)。
