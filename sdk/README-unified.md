# YMPME 一体化 SDK

一个 `lib/libympme.a` 包含全部 PME 功能：基础物理、三维刚体与关节、碰撞求解、飞行、常用模型、传感器、皮肤接触和柔性材料。默认全部接口可用，无需挑选模块或设置 feature 宏。无需 PME 源码、GRE、SDL 或 YMSKE。

## 外部工程使用

```cmake
cmake_minimum_required(VERSION 3.16)
project(my_sim C)
set(CMAKE_C_STANDARD 99)
find_package(YMPME CONFIG REQUIRED)
add_executable(my_sim main.c)
target_link_libraries(my_sim PRIVATE YMPME::ympme)
```

```console
cmake -S . -B build -DYMPME_DIR=/absolute/path/YMPME_libs/cmake
cmake --build build
```

代码只需 `#include "YMPME.h"`，也可直接包含具体模块头文件。安装后的头文件默认开启全部功能；旧的 `YMPME::dynamics`、`YMPME::encoder` 等目标作为兼容别名保留，最终均链接同一份静态库。

不用 CMake 也可以（Linux 示例）：

```console
cc -std=c99 -I /absolute/path/YMPME_libs/include/YMPME main.c /absolute/path/YMPME_libs/lib/libympme.a -lm -o my_sim
```

全部接口可用不会自动为物体创建传感器、skin 或飞行器；应用仍按需要创建对象。静态链接器通常只取程序用到的目标文件，所以一体库大小也不等于最终可执行文件增加的大小。

## 包内内容

- `lib/libympme.a`：唯一的物理静态库。
- `include/YMPME/`：匹配的一体化头文件，默认全功能。
- `cmake/`：可搬移的 CMake 导入和版本配置。
- `examples/`、`bin/`：自由落体、刚体接触力、uint16 编码器三个示例及可执行文件。
- `手册.md`、`docs/`：接口导航、坐标/内存约定和架构说明。
- `BUILD_INFO.json`、`verification/`、`checksums.sha256`：构建信息、验证记录和完整性校验。

当前平台以 BUILD_INFO.json 为准。头文件与库必须成套使用，不能和分模块包的头文件混搭。

## 重建示例和 SDK

```console
cmake -S examples -B examples-build
cmake --build examples-build
ctest --test-dir examples-build --output-on-failure
```

维护者在 PME 源码根目录运行 `python3 sdk/build.py`，默认生成 `build/YMPME_libs`。原分模块流程保留为 `python3 sdk/build_modular.py`，默认生成 `build/YMPME_libs_modular`；仅核心裁剪用 `python3 sdk/build_modular.py --profile core --output build/YMPME_core_libs`。

两个脚本共用打包验证流程，分别使用独立构建目录。只有编译、搬移后示例运行、全部目标文件链接及检查通过后，才替换旧交付包。没有桌面入口或 shell 启动脚本。
