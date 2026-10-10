# YMPME 预编译 SDK

这里存放供 Git 提交和下载的分发包。当前版本 **0.4.0**，平台 Linux x86_64，Release/PIC；包含 16／32 位索引的两份完整静态库、配套头文件、CMake 接口、终端自检示例、手册与验证记录。其他 CPU／系统需用对应工具链重建。

- [下载 SDK](YMPME_libs-linux-x86_64.tar.gz)
- [SHA-256 校验文件](YMPME_libs-linux-x86_64.tar.gz.sha256)
- [版本记录](../CHANGELOG.md)

## 解压与使用

在 `releases/` 中校验并解压到构建目录：

```console
sha256sum -c YMPME_libs-linux-x86_64.tar.gz.sha256
mkdir -p ../build/release-unpacked
tar -xzf YMPME_libs-linux-x86_64.tar.gz -C ../build/release-unpacked
```

解压后只有一个顶层目录 `YMPME_libs/`。进入该目录：

```console
sha256sum -c checksums.sha256
./bin/index16/demo_sdk_version
./bin/index32/demo_sdk_version
cmake -S examples -B examples-build16 -DYMPME_INDEX_BITS=16
cmake --build examples-build16 -j4
ctest --test-dir examples-build16 --output-on-failure
```

32 位索引另建构建目录，将 `16` 改为 `32`。自己的工程通过 `find_package(YMPME 0.4 CONFIG REQUIRED)` 接入，将 `YMPME_DIR` 指向解压目录的 `cmake/`，链接 `YMPME::ympme`。完整包功能默认可用，实例仍由应用按需创建；需要裁剪时使用源码或分模块 SDK。

物理库与纯数值示例不依赖 GRE、SKE、SDL，也不自动创建 skin。GRE 共享网格示例重编时需另行提供匹配索引位宽的 GRE SDK；该 SDK 不捆绑外部引擎、动画资源或窗口演示。详细接入见包内 `README.md`、`手册.md` 和 `docs/`。预编译程序的系统依赖以 `BUILD_INFO.json` 记录的平台为准。

## 维护分发包

在源码根目录执行：

```console
python3 sdk/package.py --release --rebuild --jobs 4
```

`--rebuild` 使用现有 SDK 构建器完成两种位宽的构建和 GRE 共享验证；可用 `--gre-sdk /absolute/path/YMGRE_libs` 指定配套 GRE。已有 SDK 与当前源码摘要一致时，可省略 `--rebuild`，直接验证和压缩。

发布脚本检查包内全部校验和、版本及源码摘要，实际解压压缩包，再为两种位宽重编并运行纯物理外部示例、检查预编译版本程序。全部通过后才替换压缩包和校验文件；构建、压缩或解压验收失败保留原发布包，最终校验文件替换失败会恢复原压缩包。相同 SDK 内容生成相同压缩包。

省略 `--release` 只写入 `build/archives/`，不更新本目录。验证日志位于 `build/.cache/release-package/consumer16.log` 和 `consumer32.log`。发布入口不会自动提交 Git、推送或创建远程 Release。相关失败回归运行 `python3 tests/test_sdk_package.py`。
