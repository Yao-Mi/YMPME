# PULLEY：滑轮提升模型

四股绳承重的 4:1 提升机构，负载 / 手柄质量、张力、保持与升降指令。

代码已从原 Demo 移到此目录；mechanisms 直接链接 **`YMPME::pulley`** 并调用同一实现。公开接口包含 `MODEL/PULLEY/YMPME_Pulley.h`，函数使用 `YMPME_` 前缀，状态类型使用 `pme_` 前缀。

配置 `-DYMPME_MODEL_PULLEY=ON` 启用，默认关闭。除物理核心外的模型依赖：无。VEHICLE 的三个依赖开关需要同时启用，否则 CMake 明确报错；其它模型可单独构建和测试，不需要 Demo/GRE/SDL。

状态由调用方持有，热路径不分配内存；显示直接读取状态或连续数组，不增加序列化、节点副本或逐节点消息。初始化参数、单位、有效范围与返回行为见头文件注释。独立实例没有共享可变全局状态，同实例仍需调用方安排读写同步。

当前固定穿绕比及演示限位，不是任意滑轮拓扑或绳子绕轮碰撞。

验证：`ctest --test-dir build-models --output-on-failure`，裁剪矩阵见根目录 `python3 tests/check_models.py`。使用示例与迁移对照见 [模型总览](../README.md)。
