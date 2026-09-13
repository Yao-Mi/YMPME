# GRE 关节控制实验台

运行 `./build/project_Demo/joint_lab/demo_joint_lab`。窗口有三个同时运行的场景：

1. **转动关节电机单摆**：复用 PENDULUM Model。应用侧 PD + 重力补偿输出力矩，最大 25 N·m；可关闭电机自由摆动，可切换 ±80° 限位。青色线表示请求目标，碰到限位时请求目标与实际角度可以不同。
2. **固定关节悬臂**：2 kg 梁在重力下固定，按 F 在末端施加额外 30 N 向下力。显示支承力、固定锚点处的反作用力矩及质心加速度。
3. **滑动关节往复平台**：2 kg 平台沿 X 轴往复，力控制输出最大 20 N，行程 ±1.2 m。导轨的支承力平衡重力，其余五个自由度锁定。

| 键 | 操作 |
| --- | --- |
| ↑ / ↓ | 单摆目标角增加 / 减少 15° |
| M / T | 电机开关 / 自动扫动目标角 |
| L / F | 铰链限位开关 / 悬臂额外载荷 |
| Space / N | 暂停 / 暂停下单个物理子步 |
| Home | 重置物理与控制状态 |
| C / V | 正视 / 立体视角，实体 / 线框 |
| Esc | 退出 |

物理固定 240 Hz，GRE 独立刷新，最多追赶 0.1 s；窗口停顿不会使物理使用大 dt。编码器按物理时间采样，16 位单圈输出；相对初始角的累积采用：

```c
uint16_t now = sample.angle, last = state.lastEncoder;
int16_t delta = (int16_t)(now - last);
state.accumulatedTicks += delta;
```

要求相邻真实转角小于半圈。内部关节坐标持续累积 rad，编码器本身保持 uint16 回绕。

反馈分别显示电机输出、关节总载荷、角速度和角加速度。单摆的 `Total joint torque at COM` 关于负载质心，包含锚点支承力矩；中间场景的 `Support moment` 关于固定锚点，两者参考点不同。单步数据为求解冲量/dt，不是独立测力硬件噪声输出。

构建：

```sh
cmake -S . -B build -DYMPME_BUILD_GRE_DEMO=ON -DYMPME_GRE_WINDOW=ON \
  -DYMPME_GRE_ROOT=../0502_YMGRE -DYMPME_MODEL_PENDULUM=ON -DYMPME_SENSOR_ENCODER=ON
cmake --build build --target demo_joint_lab -j
./build/project_Demo/joint_lab/demo_joint_lab
```

控制逻辑在 `control/`，可无 GRE/SDL 单独测试。`--smoke-test` 自动验证暂停/单步、电机、限位、载荷、视角、重置、编码器及变化的渲染帧。软件 SDL 渲染默认在窗口初始化前启用，避免此前 GLX 建上下文失败。

红色止挡和支架是可视化，机构靠真实关节约束承载；这版通用刚体尚未接入旧 World 的形状碰撞。独立模块边界见 [JOINT](../../YMPME/JOINT/README.md)。
