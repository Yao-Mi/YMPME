# 单模块示例

此目录放单模块的用法和可视化演示，可以有窗口、交互及多个源文件。
多个模块组合的完整应用场景放在 `project_Demo/`。

- `demo_free_fall.c`：外部静态缓存推进 120 个物理步，打印高度和速度。
- `demo_statics.c`：二维悬臂梁命令行算例，打印位移、转角和支座反力。
- [statics_viewer](statics_viewer/README.md)：GRE 窗口展示弹簧、梁、框架和桁架等 7 种结构的载荷、变形及内力。

- `demo_statics3d.c`：三维梁双向弯曲与扭转的命令行算例。
- [statics3d_viewer](statics3d_viewer/README.md)：GRE 三维静力窗口，双向弯曲、空间桁架及扭转三个算例。

- `demo_solid_statics.c`：四面体实体块受弯的命令行有限元算例。
- [solid_statics_viewer](solid_statics_viewer/README.md)：GRE 实体有限元窗口，立方体受压、块体弯曲、内部切片和应力/位移/应变能云图。
