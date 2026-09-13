# GRE 实时弹球

在 YMPME 根目录运行 `./build/project_Demo/gre_bouncing_ball/demo_gre_bouncing_ball`，打开窗口。复用现有 YMGRE_DemoHost / YMGUI / SDL2；物理使用真实帧间隔驱动 120 Hz 固定步，再插值生成 GRE 顶点并刷新窗口。640×420 场景加底部状态栏，显示画布为 640×500。

- Space：暂停/继续；暂停时物理时钟冻结。
- R：恢复到初始位置，清空速度并重新计时，继续运行。
- ↑：以 6 m/s 竖直向上弹起，并继续运行。
- Esc / 关闭窗口：退出并释放场景和窗口资源。

默认使用 1 倍窗口；可设 `YMGRE_WINDOW_SCALE=2`。键盘操作需要窗口获得焦点。

手动构建时设置 `-DYMPME_BUILD_GRE_DEMO=ON -DYMPME_GRE_WINDOW=ON -DYMPME_GRE_ROOT=/home/yaomi/文档/0502_YMGRE`。运行 `build/project_Demo/gre_bouncing_ball/demo_gre_bouncing_ball` 即可。

`--headless` 保留第 0/60/120/240 步 PPM 输出；应从 build 运行以将文件留在构建目录。没有 SDL2 的机器用 `-DYMPME_GRE_WINDOW=OFF` 构建纯离屏程序。

`--smoke-test` 使用 SDL 事件自动验证动画变化、暂停/继续、重置、弹起、退出。CTest 使用 SDL dummy 视频驱动运行此测试；实际窗口使用桌面的显示驱动。
