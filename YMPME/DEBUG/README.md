# DEBUG 日志与诊断（0.3.1）

包含 `DEBUG/YMPME_Log.h`，也可通过 `DEBUG/YMPME_Debug.h` 或总头 `YMPME.h` 使用。日志接口在基础库中，一体 SDK 的 16/32 位库均包含。原有 `YMPME_Debug_KineticEnergy` 保留。

## 默认行为

日志编译启用，默认将 WARNING 和 ERROR 写入 stderr。普通诊断不调用 exit/abort，不代替返回值检查；参数错误仍返回原来的错误码，分配或 AddBody 失败仍返回 NULL。成功步进和正常无碰撞不输出日志。

```c
pme_result result = YMPME_Body_Init(&body, &desc);
if (result != PME_OK) {
    /* 修正输入、跳过操作或由应用决定停止。 */
}
YMPME_LOG(PME_LOG_INFO, PME_OK, "simulation ready"); /* 默认等级过滤 */
```

等级由低到高为 `PME_LOG_DEBUG`、`INFO`、`WARNING`、`ERROR`，`PME_LOG_OFF` 禁用输出。记录包含等级、错误码、消息、源文件、行号和函数名。`YMPME_Debug_ResultString` 和 `YMPME_Debug_LevelString` 返回常量字符串，未知值有兜底说明。

已有错误码数值不变；新增 `PME_OUT_OF_MEMORY=-3` 用于分配失败的日志，`PME_ASSERTION_FAILED=-4` 用于断言日志，不改变原来指针接口的返回类型。

## 接自定义输出

```c
static void my_log(const pme_log_record* record, void* user)
{
    /* 同步消费记录；可写终端、串口，或复制到应用队列后在 GRE 界面显示。 */
    (void)record;
    (void)user;
}
/* 最低等级 DEBUG 表示接收全部等级。 */
YMPME_Debug_SetLogger(my_log, application_context, PME_LOG_DEBUG);
/* 静音；NULL 回调也会禁用输出。 */
YMPME_Debug_SetLogger(NULL, NULL, PME_LOG_OFF);
/* 恢复默认 WARNING 阈值与 stderr 输出；无 stderr 构建则恢复静音。 */
YMPME_Debug_ResetLogger();
```

SetLogger 的非法等级返回 PME_INVALID_ARGUMENT，保持此前配置。日志回调替换默认输出，不同时打印两份。回调同步执行，记录及其字符串仅保证在回调期间有效；保存到队列需要复制所需内容。

每份链接的 PME 库有一个日志配置，不在 Body/World 中增加字段。应在仿真开始前或所有日志调用停止时配置；不得并发修改配置。多线程同时记录时回调及 user 数据需自行同步；回调不得重入日志或调用会产生日志的 PME 接口。库不创建线程、文件、队列，也不主动分配日志缓冲。默认 stdio 或自定义回调本身可能阻塞、分配，实时应用应选择自己的输出方式。

## 编译裁剪与断言

| CMake 开关 | 对应 C 宏 | 默认 |
|---|---|---|
| `YMPME_ENABLE_LOGGING` | `PME_ENABLE_LOGGING` | ON / 1 |
| `YMPME_DEBUG_STDERR` | `PME_DEBUG_STDERR` | ON / 1 |
| `YMPME_ENABLE_ASSERTS` | `PME_ENABLE_ASSERTS` | OFF / 0 |

嵌入式仅用回调：构建库时设置 `-DYMPME_DEBUG_STDERR=OFF`。彻底关闭日志：`-DYMPME_ENABLE_LOGGING=OFF`；日志宏不求值任何参数，也不保留消息字面量及输出调用，公共函数仍保留空操作兼容入口和错误码字符串查询。

`YMPME_REPORT(result, message)` 用于返回错误同时记录位置：result 只求值一次且返回值不变；关闭日志后 message 不求值。直接调用函数的实参依然遵循 C 的正常求值规则。日志启用但运行时过滤/静音时，宏参数仍会求值。

`YMPME_ASSERT(condition)` 默认完全关闭且不求值条件。显式开启后，条件失败会记录 `PME_ASSERTION_FAILED` 并调用 abort；即使日志被静音或编译关闭，断言仍终止。不要把必需的副作用放进断言；外部非法参数仍通过正常返回值处理，未改成致命断言。

这些开关作用于编译库本身；只给应用定义宏不能裁掉预编译库里的日志。源码 CMake 自动传播配置，安装头保存对应默认值。默认双位宽 SDK 保持日志和 stderr 开启、断言关闭；需要其它配置时从源码构建。宏、头文件和库应匹配。

## 已接入的路径与示例

已接入 Body 初始化/设置/施力错误，基础 World 初始化、步进、容量和反馈错误，World 创建分配失败，凸网格与共享布局校验，DYNAMICS 步进校验/接触及缓存容量，以及 GRE 共享绑定校验。其余模型和传感器目前仍以各自返回值为准；本次没有给每个接口逐一加日志。

SDK 的 `demo_sdk_debug.c` 展示回调、等级过滤、错误后修正输入继续运行与静音。一体包中运行 `bin/index16/demo_sdk_debug` 或 `bin/index32/demo_sdk_debug`，终端显示诊断与 PASS。测试覆盖配置恢复、位置、非法参数、容量、内存失败回滚、编译关闭和断言独立性。
