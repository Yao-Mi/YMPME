# 角度编码器

接口和参数约定见 [YMPME_Encoder.h](YMPME_Encoder.h)；采样、坐标系、裁剪与边界见 [传感器层说明](../README.md)。

`pme_encoder_sample.angle` 为 `uint16_t` 单圈码值（0..65535），65536个刻度对应一整圈。360°与0°均输出0。`YMPME_Encoder_Init(sensor, 0)` 或传 `PME_ENCODER_COUNTS_PER_TURN` 使用16位精度；低精度配置仍按相同16位范围输出。速度仍为rad/s，以 `velocityValid` 判断有效性。

差分直接使用16位整数：`uint16_t now, last; int16_t delta = (int16_t)(now - last);`。当前 GCC 补码目标将结果解释为带符号的单圈增量；仅在输出 rad/s 时乘以 `2π/65536` 并除以采样间隔。输入仿真弧度转换成码值仍使用浮点，码值差分不再使用 `remainder()`。

## 零位与安装方向

`YMPME_EncoderDevice.h` 提供机械零位、±1方向和可重复角度噪声。`Zero` 重新标定当前机械角，下一样本速度无效，避免人为校零制造速度脉冲；应用自身累积计圈也应重置 last。原 uint16_t 单圈码值与 int16_t 回绕差分保持一致。详见[器件模式](../README.md#可选器件处理接口)。
