# YMPME 原生配套 SDK（16 / 32 位）

每种索引位宽各有一份包含全部物理功能的一体库：`lib/libympme_index16.a`、`lib/libympme_index32.a`。默认 16 位，与 GRE SDK 默认配置一致；核心源码独立构建默认仍为 32 位以兼容旧代码。

索引数组统一声明为 `PME_Index`；一体 SDK 默认 16 位，源码独立构建默认 32 位。不要把 32 位索引指针强转成 16 位指针，也不要同时链接两份库。

## 使用

```cmake
find_package(YMPME CONFIG REQUIRED)
target_link_libraries(my_app PRIVATE YMPME::ympme)
```

```console
cmake -S . -B build16 -DYMPME_DIR=/absolute/path/YMPME_libs/cmake -DYMPME_INDEX_BITS=16
cmake --build build16
```

使用 `#include "YMPME.h"`，网格索引声明为 `PME_Index`。32 位另建构建目录并设置 `YMPME_INDEX_BITS=32`。CMake 自动选取库并传递宏；同一程序不要混合两种 PME 位宽。没有模块选择步骤，所有物理接口默认可用。

不用 CMake 时：

```console
cc -std=c99 -DYMPME_INDEX_BITS=16 -I /absolute/path/YMPME_libs/include/YMPME main.c /absolute/path/YMPME_libs/lib/libympme_index16.a -lm -o my_app
```

## 运行包内示例

在本包根目录执行：

```console
./bin/index16/demo_sdk_shared_gre
./bin/index32/demo_sdk_shared_gre
```

这两个程序在内存中执行 GRE 软件渲染，并验证共享地址、碰撞与源数据不变，结果输出到终端；不会弹出窗口。其余示例及用法见 [手册](手册.md)。

## 日志诊断

默认 WARNING/ERROR 输出到 stderr，包含错误原因与函数/行号，不自动退出程序。支持自定义回调、等级过滤、静音及编译裁剪，断言默认关闭。运行 `./bin/index16/demo_sdk_debug` 查看错误后恢复示例。详见 [DEBUG 说明](docs/DEBUG.md)。

## 与 GRE 共用数据

`examples/gre/YMPME_GREMesh.h/.c` 是应用侧原生适配层。GRE 持有局部顶点和每面索引，PME 借用它们，不复制几何、不展开索引。物理轴缓存和显示工作数据各自保留，PME 的库本身不依赖 GRE。

```console
cmake -S examples -B examples-build16 -DYMPME_INDEX_BITS=16 -DYMPME_SDK_WITH_GRE=ON -DYMGRE_DIR=/absolute/path/YMGRE_libs/cmake
cmake --build examples-build16
ctest --test-dir examples-build16 --output-on-failure
```

示例构建会选用同位宽的 GRE。`demo_sdk_shared_gre` 验证真实渲染、碰撞、指针共享、源数据不变及解绑。它是无窗口自检程序，运行结果显示在终端。

绑定只支持单个闭合凸三角网格，共享边须使用相同顶点索引。凹模型、未三角化多边形、UV 接缝导致的重复顶点拓扑不会自动修复。绑定后原始几何和 scale 不可修改，不得对它调用 GRE 的 LocalToWorld；使用 `PrepareRender` 写入已有工作顶点。完整操作与所有权约定见 `docs/SHARED_MESH.md`。

## 文件和维护

- `include/`、`lib/`、`cmake/`：共享头文件、两份完整静态库、自动选择配置。
- `examples/`、`bin/index16/`、`bin/index32/`：基本物理、力反馈、编码器、共享网格及 GRE 配套示例。
- `verification/`：各位宽搬移后测试、完整链接及原生 GRE 共享验证。
- `BUILD_INFO.json`、`checksums.sha256`：版本、架构、源码摘要与文件校验。

在源码根目录运行 `python3 sdk/build.py` 重建；可用 `--gre-sdk` 指定 GRE SDK 所在目录。原分模块方式保留为 `python3 sdk/build_modular.py --index-bits 32`。

当前版本为 0.3.x，共享视图扩展了网格结构，旧 0.2.x 二进制消费者需要用新头文件重新编译。库、头文件和索引宏必须匹配。功能默认可用不会自动创建传感器或皮肤对象。

## 授权声明

个人及非商业组织以非营利目的学习、研究或自用 YMPME，可免费使用；商业用途必须事先说明并取得著作权人的书面授权。对外分发修改版或使用修改版对外提供服务时，须按许可条款公开本库的对应源码及必要构建文件。第三方资源及外部依赖按各自原许可使用。完整条款见 SDK 根目录的 `LICENSE`；源码中的许可文件位于项目根目录。
