# ROPE：可复用绳索模型

`YMPME::rope` 是独立 C99 静态库，只依赖 YMPME 核心与 libm。源码不包含 GRE、YMGUI、SDL，也不调用分配函数。默认关闭，使用 `-DYMPME_MODEL_ROPE=ON` 启用；消费者显式包含 `MODEL/ROPE/YMPME_Rope.h`。

## 已支持

- 单端固定、有质量绳身和绳端重物。
- 两端固定、绳长大于跨度时自然下垂。
- 规定运动的锚点、固定与释放节点、施加节点冲量。
- 任意三维初始节点、非均匀质量 / 绳段长度、开放链和闭合链。
- 只受拉的 XPBD 绳长约束、可选柔度、阻尼与逐段张力估计。

模型是无抗弯刚度的节点链。尚无绳身碰撞、自碰撞、缠绕、打结、滑轮穿绕或外部刚体的双向耦合；闭合链是可复用基础，原碰撞皮带尚未迁移。迭代求解存在长度误差、数值阻尼与浮点误差，非精确解析悬链线或工程应力计算器。

## 最小用法

```cmake
set(YMPME_MODEL_ROPE ON CACHE BOOL "" FORCE)
add_subdirectory(path/to/YMPME ympme-build)
target_link_libraries(my_game PRIVATE YMPME::rope)
```

```c
#include "MODEL/ROPE/YMPME_Rope.h"

/* 这些实例及缓冲必须比使用它们的仿真/显示绑定长寿。 */
pme_rope rope;
pme_rope_node nodes[33];
pme_rope_link links[32];

pme_result result = YMPME_Rope_InitSpan(
    &rope, nodes, 33, links, 32, 33,
    YMPME_Vec3(-1.5f, 1, 0), YMPME_Vec3(1.5f, 1, 0),
    4.0f, .04f, 1, 1); /* 4m 绳、3m 跨度、每节点 .04kg、固定两端 */
if (result == PME_OK) {
    result = YMPME_Rope_Step(&rope, 1.0f / 120);
    pme_rope_view view = YMPME_Rope_GetView(&rope);
    /* view.nodes 就是 nodes；读取其 position/previous，无数组拷贝。 */
}
```

绳端加重可调用 `YMPME_Rope_SetMass(&rope, rope.count-1, 2)`；使用该节点位置作为物体质心，保证初始物体就在绳端。固定端调用 `SetAnchor` 设置下一步目标；模型在本次外部 Step 的子步中插值移动到目标。此接口规定端点运动，不自动向外部刚体反馈力。

需要闭环或任意初始形状时使用 `DefaultDesc` + `Init`，描述包含位置、质量、可选逐段长度；输入数组只在初始化时读取。初始化先校验再写入，容量不足或非法输入不修改输出。节点/连接工作缓冲由调用方提供，不提供隐式堆分配。配置范围、所有权和失败行为详见头文件。

## 时钟、视图与状态

外部步长允许 `[1e-6, .05]s`，默认子步不大于 `1/600s`、24 次迭代。每次外部 Step 开始保存 previous，供渲染插值。`gravity/damping/compliance/maxSubstep/iterations` 可以在两步之间调整，Step 会校验其范围。更多节点、更悬殊的质量比需要更多迭代或更小子步；不要把测试容差理解为任意配置保证。

`GetView` 是 O(1) 借用连续节点和连接数组，包含状态 revision。视图不能与步进/改参/重置无保护并发；实例和工作缓冲不能浅拷贝后分别独立推进。拓扑在一次 Init 的生命周期内固定；重新初始化后渲染绑定须重新绑定，即使地址未变。

## 可选 GRE 适配

`YMPME::rope_gre` 放在 `BRIDGE`，单向依赖 `YMPME::rope` 和 GRE。独立使用适配层时配置：

```sh
cmake -S . -B build-rope-gre \
  -DYMPME_MODEL_ROPE=ON -DYMPME_BUILD_ROPE_GRE=ON \
  -DYMPME_BUILD_DEMOS=OFF -DYMPME_BUILD_GRE_DEMO=OFF \
  -DYMPME_GRE_ROOT=/home/yaomi/文档/0502_YMGRE
```

该配置不构建窗口或依赖 SDL。开启绳索相关 GRE Demo 时会一并构建适配目标。

`YMPME_GRE_Rope_Create` 在初始化时创建一份固定拓扑网格，每绳段 8 顶点 / 12 三角形。`Sync` 直接从物理数组插值、转换单位、写入顶点和法线；复用网格，不生成整条绳的状态副本。revision 与 alpha 均未变化时跳过写入。同拓扑重置使用 `Rebind` 使缓存失效并复用网格，改变节点数 / 闭合状态时需要 Destroy 后重新 Create。

适配网格使用方形截面，radius 为截面半宽；节点视觉接缝不代表碰撞几何。采用普通顶点和平面法线，不提供平滑着色。网格与模型分属不同所有者，由适配层 Destroy 释放网格，应用仍拥有模型缓冲。

## 演示与验证

运行根目录 `./build/project_Demo/rope_lab/demo_rope_lab`。三幅 GRE 视图展示固定端下垂、17 节点绳端挂物、33 节点三维移动锚点；窗口显示物理步进、同步和绘制耗时。原机械演示的单绳摆锤也已使用此库的两节点实例。

`ctest --test-dir build -R 'rope|mechanisms' --output-on-failure` 检查绳长、下垂、摆动能量、松弛、静态张力、柔度、移动 / 释放锚点、闭环、输入拒绝、视图、GRE 插值、缓存和窗口交互。Linux GCC/Clang 测试通过包装 malloc/calloc/realloc 验证步进与同步循环没有堆分配。

`python3 tests/check_features.py` 检查 7 组核心 / 飞行 / 绳索裁剪组合；未启用的源文件和符号不存在，核心库归档与 Body/World 布局保持一致，绳索库没有分配或渲染依赖。

性能数据和构建条件见 [性能记录](PERFORMANCE.md)。
