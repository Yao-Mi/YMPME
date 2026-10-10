# 回归与稳定性验证

测试数量随配置变化。所有物理数值测试使用显式检查，Release/NDEBUG 下仍会执行。当前发布配置无 GRE/SKE/SDL 或网络依赖；GRE 窗口测试由应用构建单独启用。

## 重复验证

CMake 3.21 以上可以使用仓库预设（源码仍可由 CMake 3.16 构建）。所有输出位于 `build/verification/`。按需将下面的 `headless` 替换为 `core`、`headless16` 或 `sanitize`：

```console
cmake --preset headless
cmake --build --preset headless -j4
ctest --preset headless -j4
```

| 配置 | 范围 |
|---|---|
| core | 核心默认裁剪，32 位索引 |
| headless | 全部纯物理模块、传感器和静力，Release / 32 位 |
| headless16 | 相同全模块，16 位索引 |
| sanitize | 相同全模块，Debug / ASan + UBSan，错误立即失败 |

`ctest --preset headless -L stability --output-on-failure` 单独运行稳定性回归。Sanitizer 默认保留泄漏检查；平台确实不支持时可另设环境并在结果中注明，不能将跳过的检查记为通过。

## 稳定性场景

- 四个 1 kg、边长 1 m 的盒体堆叠 60 秒；16 次迭代，步长按 120/240/480 Hz 切换。持续检查有限状态、姿态、速度、高度与横向漂移，同时以相同初态重复运行验证同一构建的结果一致。阈值为高度误差 <20 mm、速度 <0.25 m/s、横向漂移 <10 mm。
- 清空接触缓存并将物体瞬移分离，验证旧冲量不再产生作用。
- 八节关节链 240 Hz、4 次迭代、关节热启动运行 60 秒，最大锚点误差 <3 mm。
- 极小/非法步长、时间精度耗尽、单刚体数值溢出：返回失败且不提交状态。已有测试继续覆盖两类缓存/接触容量不足、电机限位重配、CCD 冲击容量与失败回滚。

同一构建的重复结果不承诺跨 CPU、编译器或浮点优化的逐位一致；这些场景也不保证任意刚度、质量比或长链稳定。应用应按尺寸选择容差，使用固定子步，监测误差和返回值。

## 其他回归

核心涵盖碰撞、守恒、过滤、生命周期、帧累积与分配失败；可选模块覆盖刚体、关节、电机、缓存、凸 Mesh、CCD、车辆与传动、飞行控制、空间查询、传感器、图集、采样皮肤和软材料。静力测试对照解析解、反力/能量与网格加密，并拒绝欠约束、退化、容量不足和未收敛结果。

`ympme_version` 对照公开宏、链接库版本与 CMake 版本；SDK 的 `demo_sdk_version` 在搬移后的外部工程重复检查。`sdk/build.py` 和 `sdk/build_modular.py` 在链接、组件和校验和验证全部通过后才替换旧包。

GRE 当前配置使用 `ctest --test-dir build --output-on-failure`；软指完整夹持使用已有独立 `build/soft_finger_sanitize` 配置验证。历史结果见 [HISTORY.md](HISTORY.md)。

本次验证的配置、结果与测量见 [VALIDATION.md](VALIDATION.md)。
