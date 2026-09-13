# SOFTBODY：可裁剪的局部软垫

当前实现 `YMPME_SoftPad` 是局部黏弹性软垫：每个采样点有法向压缩状态和切向摩擦弹簧。接触力会参与物体运动，卸载后压缩逐步恢复。它不等同于完整体积软体求解器。

## 构建与依赖

根工程设置 `YMPME_ENABLE_SOFTBODY=ON`，消费者链接 `YMPME::softbody`；默认 OFF。
该目标传递 `PME_ENABLE_SOFTBODY=1`，也可直接包含 `SOFTBODY/YMPME_SoftPad.h`。
依赖仅为 PME 核心数据类型/数学，不依赖 RIGID、DYNAMICS、JOINT、SENSOR、SKIN、SKE、GRE；内存由调用者提供，Step 不分配内存。

```cmake
set(YMPME_ENABLE_SOFTBODY ON CACHE BOOL "" FORCE)
add_subdirectory(path/to/YMPME pme)
target_link_libraries(my_sim PRIVATE YMPME::softbody)
```

## 每个物理子步

1. 从刚体姿态或 **SKE 已蒙皮表面**生成 `rest`：未压陷的世界坐标、朝外单位法向、表面速度、固定参考面积。材料节点和采样点的对应关系应保持稳定。
2. 应用的几何查询填充 `samples`：沿软垫法向向内退让多少才能接触障碍物，以及障碍物朝向软垫的外法向、接触点速度。障碍物无接触填 `hit=0`；整个数组可传 NULL 表示卸载。
3. `YMPME_SoftPad_Step` 输出节点压陷、世界位置和软垫受力，及双方关于各自参考点的力矩。首次使用将 nodes 清零。
4. 将双方 wrench 各施加一次，再推进刚体/骨骼动力学；不要重复积分同一个物体。多个障碍物时由应用完成最近表面选择及每个障碍物的反力归属，可按节点力自行归集。
5. 按需把节点力接到传感器，把位置接到渲染表面。返回位置已经在世界坐标，不应再次套用骨骼变换。

SKE 仍负责动画/骨架/蒙皮；PME 负责局部物理压陷；GRE 负责最终显示。第一版 demo 复用刚体夹爪，没有修改 SKE 或人体/手模型。

## 材料约定和限制

单位为 SI。`stiffness`、`shearStiffness` 是面密度刚度 Pa/m；`damping`、`shearDamping` 是 Pa·s/m。`maxCompression` 单位 m。参考面积不随形变更新。`sampledArea` 为当前受力样本参考面积总和，并非精确几何接触面积。

法向 Kelvin–Voigt 关系为 `F_axis = area * max(0, k*q + c*dq/dt)`；自由恢复采用后向欧拉 `q_new = q_old*c/(c+k*dt)`。障碍物退出时只允许推力，不产生拉力。法向接触方向与软垫朝外法向应相反，要求 `-dot(obstacleNormal,padNormal) >= 0.15`，应用应忽略或另行处理擦边接触。

切向弹簧/阻尼力受库仑摩擦限幅，并随支撑体旋转更新方向。切向状态用于摩擦，当前可视形变只有法向压陷。输出力是本构采样值，不是接触冲量除以时间后的平均值。

这是无质量、节点间互不耦合的弹性基础模型：没有体积保持、泊松鼓起、组织惯性、撕裂、邻接传播、自碰撞或 CCD。弹簧与刚体使用分步耦合，刚度/质量/时间步需匹配；材料的稳定自由恢复不代表任意刚度下的整个系统都稳定。demo 使用 240 Hz。

压缩超过极限通过 `bottomedOut` 报告；模块不会自动创造刚性背板。应用仍需背板碰撞。demo 保留原夹爪的刚性碰撞体，软垫位于背板外侧 20 mm。点采样不会给出网格级不穿透保证。

验证入口：`project_Demo/soft_finger` 的无窗口测试覆盖解析载荷、回弹、无吸引、摩擦限幅、力矩守恒、参数失败回滚，以及三种物体的实际夹持。

## SoftBall 整体形变模式

`YMPME_SoftBall.h` 提供独立的体积保持椭球模式：`rx=R-q`，`ry=rz=R/sqrt(1-q/R)`。参数刚度为 N/m、阻尼为 N·s/m（与 SoftPad 的面密度参数不同）。`SoftBall_Mode` 给出当前位置对 q 的导数，应用以接触力和该导数的点积累加广义力。`Step` 使用后向欧拉更新无质量黏弹性响应，输入非法时保留原状态。

压缩范围限制为 `[-0.25R,0.55R]`，防止退化，边界处代表模式限位；没有组织惯性、独立局部顶点动力学或邻接耦合。该模式不负责碰撞、刚体积分、骨架或显示，不分配内存。调用者应使用同一形变更新显示、碰撞、表面速度和惯量；分步耦合不代表任意大时间步的整个系统都稳定。
