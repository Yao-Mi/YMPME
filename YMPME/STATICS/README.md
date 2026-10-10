# 二维线性静力分析（可裁剪）

`YMPME_ENABLE_STATICS` 默认关闭。开启后提供 `YMPME::statics` 和
`PME_ENABLE_STATICS=1`，通过 `YMPME.h` 或 `STATICS/YMPME_Statics.h` 使用。
无需 RIGID、DYNAMICS、skin、传感器、SKE 或 GRE。仅依赖 PME 基础库与数学库。

## 支持范围

| 单元 | 输入 | 输出 |
|---|---|---|
| 轴向弹簧 | 刚度 k（N/m） | 伸长对应的轴向力 |
| 二维杆 / 桁架 | 弹性模量 E、截面积 A | 拉压力、轴向应力 N/A |
| 二维 Euler–Bernoulli 梁 / 框架 | E、A、截面惯性矩 Iz | 轴力、剪力、两端弯矩、轴向应力 |

所有节点都有 ux、uy、rz 三个自由度，输出位移、转角和反力。
XY 平面，+Y 向上，+Z 方向的逆时针转角 / 力矩为正；使用 SI 单位和 double。
梁节点共享转角意味着刚接；当前不提供梁端铰释放。

这是小位移、小转角、线弹性静力有限元：组装 K，求解 Ku=f，再恢复单元内力。
不含三维实体、壳、几何 / 材料非线性、屈曲、塑性、动力积分或接触求解。
Euler–Bernoulli 梁忽略剪切变形，适用于细长梁。当前仅支持节点力和节点力矩；
没有自动重力或分布载荷，也没有分布载荷的单元固端力恢复。
`axial_stress` 只表示 N/A，不包含梁截面的弯曲应力；若截面几何已知，可另由
截面位置计算 N/A ± M*c/I，不能将该字段称为完整应力或 von Mises 应力。

## 使用与内存

1. 填写 `pme_statics_node[]`：原始坐标、节点载荷、约束位掩码。
   `fixed` 里的位分别对应 X、Y、RZ；`prescribed[]` 支持非零支座位移 / 转角。
   未约束的 prescribed 分量必须为零。
2. 填写 `pme_statics_element[]`：连接节点及相关单元的材料 / 截面参数。
   结构体建议清零；不相关的材料字段忽略。单元原始长度必须大于零。
3. 为节点结果、单元结果和求解工作区提供互不重叠的缓冲区。
   `pme_statics_workspace_doubles(node_count)` 返回所需 double 数量，零表示
   节点数无效或容量计算溢出。调用者负责分配；库内无 malloc。
4. 调用 `pme_statics_solve()`，只有返回 `PME_STATICS_OK` 时结果有效。

求解器使用对角缩放和 Cholesky 分解。n=3*节点数，工作区需 `2*n*n+3*n`
个 double，复杂度 O(n³)，面向小结构；100 个节点约需 1.45 MB 工作区。
无梁节点的无载转角自动取零。自由平移、未支承的力矩或数值病态结构返回
`PME_STATICS_SINGULAR`，不会通过虚构支座或刚度让不稳定结构“算过去”。
数值溢出返回 `PME_STATICS_NUMERIC_FAILURE`。

节点反力为 `K*u-f`，自由自由度上的值是求解残差。
单元局部 +x 从 a 指向 b，+y 为其逆时针垂线。
`end_force` 顺序是 `[Fx_a,Fy_a,Mz_a,Fx_b,Fy_b,Mz_b]`，表示施加到单元上的端力；
若需要单元施加到节点上的力，应取反。`axial_force` 拉伸为正、压缩为负。

## 构建与验证

在仓库根目录运行：

```text
cmake -S . -B build/statics -DYMPME_ENABLE_STATICS=ON -DYMPME_BUILD_TESTS=ON -DYMPME_BUILD_DEMOS=ON
cmake --build build/statics -j
ctest --test-dir build/statics --output-on-failure
./build/statics/demo_statics
```

`Demo/demo_statics.c` 是独立命令行算例：长 2 m 的钢制悬臂梁，
20×40 mm 矩形截面，端部向下 10 N。理论端部挠度 -1.25 mm、转角
-0.0009375 rad、支座 Fy=10 N、Mz=20 N m。
测试覆盖非零支座位移、桁架、旋转后的悬臂梁、端部力矩、简支梁、刚体转动、
欠约束结构、非法参数和工作区容量。

启用 `YMPME_INSTALL_SDK=ON` 后，安装包会导出可选 `statics` 组件。
消费端 `find_package(YMPME CONFIG REQUIRED COMPONENTS statics)` 后链接
`YMPME::statics`。关闭模块时不会编译 / 链接静力求解器。

## 与运动仿真、显示的边界

目前通过调用者填写节点载荷进行分析，尚未自动从 DYNAMICS 采集载荷。
将 PME 载荷快照用于分析时，应将力和力矩一致地映射到结构节点；运动中的载荷
只有在适当的准静态假设下才可直接使用，也可由上层构造含惯性载荷的分析工况。
无需给物体添加皮肤传感器。GRE 可以读取节点位移和单元内力进行可视化，求解器
不依赖显示接口。纯数值算例位于 `Demo/demo_statics.c`；可选
[GRE 窗口示例](../../Demo/statics_viewer/README.md) 位于 `Demo/statics_viewer/`，
通过 `YMPME_BUILD_STATICS_VIEWER=ON` 开启。

梁和杆的线性刚度关系参考 [TU Delft 的 Euler–Bernoulli 单元讲义](https://oit.tudelft.nl/CIEM5000/2026/lecture1/other_elements.html)。
本模块统一使用逆时针 rz 为正的转角约定（与该讲义的弯曲转角符号不同）。

三维扩展位于同目录，可独立开启 `YMPME_ENABLE_STATICS3D`；见 [三维静力](README3D.md)。

三维实体四面体有限元可独立开启 `YMPME_ENABLE_SOLID_STATICS`；见 [实体有限元](README-solid.md)。
