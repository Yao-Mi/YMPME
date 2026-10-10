# Demo 截图记录

原有图片于 2026-09-14 采集，有限元图片于 2026-09-20 补充（12 个算例，另保留一张切面采集记录）；均使用本地已有的 YMPME demo 可执行文件，全部来自实际 GRE 渲染。通过 GRE DemoHost 的离屏显示后端运行，再由 `YMGUI_SHOT` 保存完整窗口帧；保留三维场景、数值读数和操作界面。仅将 BMP 无损转换为 RGB PNG 并优化压缩，没有裁切、重绘或添加模拟数据。

截图没有重编译物理代码。运行参数和可执行文件 SHA-256 记录在 [capture_manifest.json](capture_manifest.json)，用于区分截图时的构建与后续源码。清单顶层日期为首批采集日期，新增图片以各自的 `captured_on` 为准。正常模式按实际时间推进，在不同机器上运行相同帧数不保证得到完全相同姿态；界面的 FPS、耗时和载荷只是该次运行的读数。

## 复现

从仓库根目录执行，例如截取齿轮窗口：

```sh
YMGRE_HEADLESS=1 YMGRE_WINDOW_SCALE=1 YMGRE_MAX_FRAMES=90 \
  YMGUI_SHOT=/tmp/gear_mesh.bmp \
  ./build/project_Demo/gear_mesh/demo_gear_mesh
```

再用图片工具将 BMP 转存为 PNG。没有 `YMGRE_MAX_FRAMES` 时程序持续运行；`YMGUI_SHOT` 在正常退出时写出最后一帧。

| 图片 | 帧数限制 | 附加参数 / 状态 |
| :--- | ---: | :--- |
| [gear_mesh.png](gear_mesh.png) | 90 | 默认三组啮合并行 |
| [tracked_vehicle.png](tracked_vehicle.png) | 100 | `--crest`，横跨坡顶 |
| [soft_finger.png](soft_finger.png) | 3 | `--snapshot`，程序内置的 6 秒夹持暂停状态 |
| [hand_ball.png](hand_ball.png) | 110 | 默认抓握与手掌运动 |
| [human_ground.png](human_ground.png) | 130 | 默认行走和全身拓扑图 |
| [owl_flight.png](owl_flight.png) | 100 | 默认飞行和拍翅 |
| [sensor_lab.png](sensor_lab.png) | 90 | 默认雷达、深度与触觉场景 |
| [flight_playground.png](flight_playground.png) | 100 | 默认自动飞行，独立 PID 控制四旋翼 |
| [vehicle_playground.png](vehicle_playground.png) | 30 | 默认三车近景，显示四轮悬挂压缩量 |
| [track_lab.png](track_lab.png) | 120 | 默认双轮履带台架，电机正转与弹簧张紧 |
| [mechanisms.png](mechanisms.png) | 100 | 默认四机构自动往返 |
| [rope_lab.png](rope_lab.png) | 0 | `--smoke-test`；完整窗口自检在第 110 帧退出，截图为重置后再次释放的状态 |
| [joint_lab.png](joint_lab.png) | 100 | 默认三类关节并行 |
| [grasp_lab.png](grasp_lab.png) | 170 | `--convex`；自动夹持削角凸体 |
| [human_skin.png](human_skin.png) | 85 | 默认人体自接触与滑动 |
| [hand_skin.png](hand_skin.png) | 85 | 默认掌心探头按压 |
| [three_wheel_car.png](three_wheel_car.png) | 20 | 默认驾驶场景 |
| [gre_bouncing_ball.png](gre_bouncing_ball.png) | 35 | 默认重力弹跳 |
| [ccd_lab.png](ccd_lab.png) | 34 | `--smoke-test`；首个物理步后暂停；为截帧提前结束自检，不代表完整测试通过 |
| [statics2d.png](statics2d.png) | 5 | `--scene 1`；二维悬臂梁，弯矩着色 |
| [statics3d.png](statics3d.png) | 5 | `--scene 3`；三维悬臂梁纯扭转 |
| [solid_statics.png](solid_statics.png) | 5 | `--scene 2`；块体弯曲，默认平滑 von Mises 云图 |
| [solid_statics_cut.png](solid_statics_cut.png) | 5 | `--scene 2 --cut --shading element`；同一块体的内部切面与原始单元应力 |
| [statics2d_truss.png](statics2d_truss.png) | 5 | `--scene 2`；2 · 三角桁架 |
| [statics2d_springs.png](statics2d_springs.png) | 5 | `--scene 3`；3 · 串联弹簧 |
| [statics2d_simple_beam.png](statics2d_simple_beam.png) | 5 | `--scene 4`；4 · 简支梁 |
| [statics2d_continuous_beam.png](statics2d_continuous_beam.png) | 5 | `--scene 5`；5 · 两跨连续梁 |
| [statics2d_frame.png](statics2d_frame.png) | 5 | `--scene 6`；6 · 门式框架 |
| [statics2d_bridge.png](statics2d_bridge.png) | 5 | `--scene 7`；7 · Warren 桥式桁架 |
| [statics3d_bending.png](statics3d_bending.png) | 5 | `--scene 1`；1 · 双向弯曲悬臂梁 |
| [statics3d_truss.png](statics3d_truss.png) | 5 | `--scene 2`；2 · 空间四杆桁架 |
| [solid_statics_cube.png](solid_statics_cube.png) | 5 | `--scene 1`；1 · 立方体压缩 |

首页按二维 7 个、三维杆梁 3 个、实体 2 个算例展示，各一张图。`solid_statics_cut.png` 仅保留为切面采集记录，不再在首页重复展示块体。

有限元截图使用 `build/Demo/<名称>/demo_<名称>` 下的程序，例如：

```sh
YMGRE_HEADLESS=1 YMGRE_WINDOW_SCALE=1 YMGRE_MAX_FRAMES=5 \
  YMGUI_SHOT=/tmp/solid_statics_cut.bmp \
  ./build/Demo/solid_statics_viewer/demo_solid_statics_viewer --scene 2 --cut --shading element
```

有限元截帧均正常退出；这属于画面采集，不代表重新运行了完整数值测试。所有变形倍率仅用于显示，原始计算数值保留在侧栏。

`YMGRE_MAX_FRAMES=0` 表示不限制帧数。绳索截图来自完整窗口自检；CCD 截图为固定步自检的中途帧，提前停止产生的非零退出码已记录，不将其计为完整测试通过。

## 场景资源

手掌、人体和猫头鹰由 demo 读取相邻 YMSKE 工程的资源，图片不包含模型源文件。具体依赖见各 demo 的 README。

猫头鹰资源的本地说明位于 `YMSKE/Resource/owl/README.md`：原模型来自 [Blend Swap / Owl](https://blendswap.com/blend/7951)，上传者 gholm；模型 Tristan Lock，羽毛、绑定及动画 Jeremy Davidson，灯光、毛发、材质、贴图与合成 Matt Ebb。YMSKE 原资源包保留了 CC0 许可文件及完整转换说明。
