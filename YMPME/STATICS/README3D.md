# 三维线性静力分析

`YMPME_ENABLE_STATICS3D=ON` 提供独立的 `YMPME::statics3d` 目标和
`PME_ENABLE_STATICS3D=1` 宏。默认关闭，可以与二维 STATICS 分别裁剪。
代码仍放在 `YMPME/STATICS/`。模块仅链接 PME 基础库与数学库，不依赖
GRE、SKE、DYNAMICS、传感器或二维求解器。状态和单元枚举复用二维公共头。

## 范围和单位

支持空间轴向弹簧、杆/桁架、Euler–Bernoulli 梁/刚接框架。每节点六自由度，
顺序为 `ux,uy,uz,rx,ry,rz`，载荷顺序为 `Fx,Fy,Fz,Mx,My,Mz`。
世界坐标右手系；力 N、长度 m、力矩 N·m、转角 rad，全部使用 double。

梁输入 E、A、G、J、Iy、Iz 和截面方向 `local_y`；局部 x 从节点 a 指向 b，
局部 y 为输入参考向量在法平面的投影，局部 z=x×y。方向与轴线平行时返回
INVALID_ARGUMENT，不猜测梁截面的朝向。J 为截面扭转常数，不可对任意截面
直接用 Iy+Iz 替代。轴向杆/弹簧的横向坐标轴由库生成，不影响轴向结果。
`pme_statics3d_element_frame()` 输出局部三轴，方便显示模块直接对接。

使用小位移、小转角线性静力假设，支持非零支座位移/转角与节点力/力矩。
梁含两个方向的弯曲、轴向伸长及扭转。共享梁节点表示刚接；没有梁端释放。
不含三维实体/四面体、壳、剪切变形、分布载荷、自动重力、屈曲、塑性、
接触/脱离或动态振动。约束为双向约束，可能需要抗拔支承。

结果包括全局节点位移及反力 `K*u-f`，局部单元两端的力和力矩。
`axial_force` 拉正压负，`axial_stress` 仅是 N/A，不是总应力或等效应力。
两端内力顺序为 `[Fx,Fy,Fz,Mx,My,Mz]` at a，再 at b，表示施加到单元上的力。

## 内存与调用

调用者提供节点、单元、输出及工作区，不能互相重叠。库不分配堆内存。
令 n=6*节点数，工作区为 `2*n*n+3*n` 个 double，通过
`pme_statics3d_workspace_doubles()` 查询；存储 O(n²)，求解 O(n³)，面向小结构。
未参与梁单元的无载转动自由度自动取零，未约束平移、机构或不受支承的力矩
返回 SINGULAR。只在返回 OK 时使用结果。

```sh
cmake -S . -B build/statics3d -DYMPME_ENABLE_STATICS3D=ON
cmake --build build/statics3d --target demo_statics3d test_statics3d -j
./build/statics3d/demo_statics3d
ctest --test-dir build/statics3d -R ympme_statics3d --output-on-failure
```

纯数值例子：`Demo/demo_statics3d.c`。带 GRE 的窗口例子见
[statics3d_viewer](../../Demo/statics3d_viewer/README.md)。
SDK 可用 `find_package(YMPME CONFIG REQUIRED COMPONENTS statics3d)` 后链接
`YMPME::statics3d`，不必启用二维静力或显示模块。

验证涵盖双向弯曲、扭转、任意空间朝向、截面旋转、空间桁架、支座移动及
转动、全局力/力矩平衡、欠约束、非法参数与工作区容量。
三维梁参数及坐标变换的参考：
[OpenSees elasticBeamColumn](https://opensees.github.io/OpenSeesDocumentation/user/manual/model/elements/elasticBeamColumn.html)、
[Linear Transformation](https://opensees.github.io/OpenSeesDocumentation/user/manual/model/geomTransf/Linear.html)。
本库使用局部 y 参考向量，OpenSees 上述接口使用 x-z 平面参考向量，两者接口不同。
