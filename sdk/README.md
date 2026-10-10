# YMPME 分模块 SDK

可搬移的 C99 物理引擎开发包。包含核心与可选模块的独立静态库、匹配头文件、CMake 导入配置、使用手册及按模块启用的无窗口示例。无需 PME 源码、GRE、SDL 或 YMSKE。

| 目录 | 内容 |
|---|---|
| `include/YMPME/` | 核心及可选模块头文件，保持原有目录结构 |
| `lib/` | 分模块静态库；只链接需要的目标及其必要依赖 |
| `cmake/` | `find_package(YMPME CONFIG REQUIRED)` 配置及版本信息 |
| `examples/` | 版本检查、自由落体、刚体接触、编码器、静力、共享网格及可选 GRE 共享示例 |
| `bin/` | 本机构建的无窗口示例可执行文件 |
| `docs/`、`手册.md` | API、架构、碰撞说明和 SDK 使用方法 |
| `BUILD_INFO.json` | 平台、编译器、构建选项、源码摘要 |
| `verification/` | 搬移后外部链接与运行验证记录 |
| `checksums.sha256` | 包内文件校验和 |

预编译文件适用于 BUILD_INFO.json 记录的平台；其他 CPU、操作系统和 MCU 需使用对应工具链重新构建。完整包提供所有模块，但链接核心或动力学不会自动启用传感器、skin 或飞行。

## 在外部工程里使用

```cmake
cmake_minimum_required(VERSION 3.16)
project(my_sim C)
set(CMAKE_C_STANDARD 99)
find_package(YMPME 0.4 CONFIG REQUIRED COMPONENTS dynamics)
add_executable(my_sim main.c)
target_link_libraries(my_sim PRIVATE YMPME::dynamics)
```

```console
cmake -S . -B build -DYMPME_DIR=/absolute/path/YMPME_libs/cmake
cmake --build build
```

也可使用 `-DCMAKE_PREFIX_PATH=/absolute/path/YMPME_libs`。`#include "YMPME.h"` 即可访问目标启用的接口，依赖库与 feature 宏由 CMake 自动传递。

## 运行和重编示例

直接运行 `bin/demo_sdk_minimal`、`bin/demo_sdk_dynamics`、`bin/demo_sdk_encoder`，输出在终端。它们无需外部资源。

```console
cmake -S examples -B examples-build
cmake --build examples-build
ctest --test-dir examples-build --output-on-failure
```

核心裁剪包提供自由落体、共享网格索引和日志示例；其他示例按已有模块自动启用。没有桌面入口或 shell 启动包装。

维护者在 PME 源码根目录运行 `python3 sdk/build_modular.py` 可重新打包；包使用者无需此脚本。模块选择、所有目标名、内存与坐标约定、GRE 对接方式见 `手册.md`。

共享网格与索引位宽详见 docs/SHARED_MESH.md。分模块脚本可指定 --index-bits 16 或 32。

日志接口与编译开关见 [DEBUG 说明](docs/DEBUG.md)，默认 WARNING/ERROR 输出到 stderr，普通错误不终止进程。

## 带调试信息构建

`python3 sdk/build.py --build-type RelWithDebInfo` 生成保留优化和源码级调试信息的一体 SDK；
`--build-type Debug` 适合单步调试，省略参数仍为 `Release`。分模块入口也支持此参数。
构建类型记录在 `BUILD_INFO.json` 中。日志和断言开关独立于调试符号。
调试库内部源码时需保留对应版本的源码；在其他机器可用调试器配置源码路径映射。
外部依赖保留各自的构建配置，不会因此自动获得调试符号。

## 授权声明

个人及非商业组织以非营利目的学习、研究或自用 YMPME，可免费使用；商业用途必须事先说明并取得著作权人的书面授权。对外分发修改版或使用修改版对外提供服务时，须按许可条款公开本库的对应源码及必要构建文件。第三方资源及外部依赖按各自原许可使用。完整条款见 SDK 根目录的 `LICENSE`；源码中的许可文件位于项目根目录。
