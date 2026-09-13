# Soft finger：柔软指腹夹持实验

复用 `grasp_lab` 的双指夹爪、升降关节、刚体动力学和 GRE 场景，新增两个 8×16 软垫。不会改动原 grasp demo。夹爪夹住球、方块或倒角凸块后升起，约 7 秒时松开，11 秒循环。

- 材料在 `YMPME/SOFTBODY`，默认可裁剪，独立于传感器、骨架和渲染。
- 只有夹爪指腹配置触觉；球使用体积保持的黏弹性形变模式；方块和倒角块仍为刚体。球没有添加触觉传感器。
- 上方两张图显示参考面积上的平均法向压力（0～2 kPa）；下方显示法向压陷（0～10 mm）。颜色超出图例范围会饱和，数值仍保留真实最大值。
- 3D 网格以真实尺寸显示法向形变，网格顶点取相邻材料采样压陷的平均值。切向弹簧用于摩擦，不额外移动显示顶点。
- 20 mm 软垫有刚性背板；状态栏显示压到底的采样数。几何适配器是局部射线采样，支持形变后的椭球及此 demo 的凸网格，不是完整软体碰撞求解器。

## 运行

编译好的可执行文件：`build/project_Demo/soft_finger/demo_soft_finger`。可直接双击，运行不依赖当前工作目录。无桌面入口，无启动脚本，窗口统一走 GRE DemoHost。

- `1 / 2 / 3`：球 / 方块 / 倒角块，重置实验。
- `K`：指腹柔软 / 较硬材料，并重置。较硬指腹刚度为三倍。
- `B`：球特别软 / 较硬，并重置。球的刚度分别为 40 / 600 N/m。
- `T`：自动 / 手动；`A / D` 合拢 / 张开；方向键上 / 下升降。
- `F`：摩擦开关；`Space` 暂停；`N` 推进一个物理子步。
- `S`：0.1 倍慢放；`R` 重置；`C` 切换视角；`V` 被夹物线框；`Esc` 退出。

`--snapshot` 从自动夹持第 6 秒暂停开始，方便观察。`--smoke-test` 自动验证输入、暂停/单步、抬升、释放及材料/形状切换。

## 独立构建

从 YMPME 根目录执行；所有产物在 build 内：

```console
cmake -S project_Demo/soft_finger -B build/soft_finger -DSOFT_FINGER_WINDOW=ON -DYMPME_GRE_ROOT=/home/yaomi/文档/0502_YMGRE -DCMAKE_BUILD_TYPE=Release
cmake --build build/soft_finger --target demo_soft_finger test_soft_ball test_soft_pad test_soft_finger -j4
ctest --test-dir build/soft_finger --output-on-failure
```

无窗口 / 无 GRE 构建：

```console
cmake -S project_Demo/soft_finger -B build/soft_finger_headless -DSOFT_FINGER_WINDOW=OFF
cmake --build build/soft_finger_headless --target test_soft_ball test_soft_pad test_soft_finger -j4
ctest --test-dir build/soft_finger_headless --output-on-failure
```

可加 `-DSOFT_FINGER_SANITIZERS=ON` 检查地址/未定义行为。该独立 CMake 工程只定义自己的测试，不改用主工程已有 build 配置。

## 软球形变

球采用 `YMPME_SoftBall` 的单模态仿射形变：本体 X 方向压缩，Y/Z 等比例鼓起，三个缩放系数乘积为 1。默认夹持稳定后，X 宽度约为原来的 56%，Y/Z 约为 133%，为真实比例。自动松开后回弹；落地仍有重力/接触载荷，所以不要求落地时完全恢复正球。

该形变由接触力的广义力驱动，恢复使用弹簧和阻尼；查询使用同一个椭球方程，刚性背板和地面接触使用同步更新的 26 顶点凸网格代理。网格为低模近似，不能保证解析表面处处零穿透。形变使用上一子步的接触力，维持 240 Hz 的分步耦合；碰撞体惯量同步更新并保留更新时的角动量。

它是方便观察的整体软球近似，不是 FEM、任意局部凹陷或全三维软体求解器；当前形变模式围绕本体 X 轴，旋转后跟随物体。默认相机已拉近。
