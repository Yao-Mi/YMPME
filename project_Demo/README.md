# 完整流程示例

- `bouncing_ball`：无窗口固定时钟、弹跳、渲染插值，输出 CSV 到标准输出。
- `gre_bouncing_ball`：链接真实 YMGRE，在 YMGUI/SDL 窗口实时显示弹跳，支持暂停、重置、再次弹起；--headless 可输出 PPM 帧。

构建和运行方式见各目录 README。

- `three_wheel_car`：两差速轮 + 后万向轮的可驾驶小车，障碍碰撞、五个检查点、双视角。

- `vehicle_playground`：履带、三轮、四轮车同场驾驶，独立弹簧阻尼悬挂、共享几何高度场、PGM 地图和自动过山包测试。

- `gear_mesh`：20/30 齿渐开线三组对照：理想约束、紧间距逐齿碰撞和加大间距回差碰撞；真实接触冲量、低速自动反转、暂停单步与啮合区近景。

- `mechanisms`：四分屏机械演示，约束皮带与接触摩擦皮带、4:1 滑轮组提升重物、绳端物体自由摆动；支持暂停、单步和摩擦开关。

- `rope_lab`：复用 MODEL/ROPE，同屏展示两端固定下垂、有质量绳身挂物和三维运动锚点，显示物理/同步/绘制耗时。

- `flight_playground`：无人机和老鹰飞行场，调用可裁剪 AIRCRAFT / FLIGHT 模块；`./build/project_Demo/flight_playground/demo_flight_playground` 启动。

- `owl_flight`：猫头鹰原模型与拍翅动画，键盘第三人称飞行、滑翔和自动盘旋；运行 `build/project_Demo/owl_flight/demo_owl_flight`。见 [操作和独立构建说明](owl_flight/README.md)。
