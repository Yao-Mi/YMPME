# 接触测力与载荷变换

接口和参数约定见 [YMPME_ForceSensor.h](YMPME_ForceSensor.h)；采样、坐标系、裁剪与边界见 [传感器层说明](../README.md)。

## 六维器件模式

`YMPME_ForceDevice.h` 增加六轴偏置、力/力矩噪声、量程/量化/滤波与下一成功样本去皮。输入是世界坐标、关于质心的 `pme_force_sample`，保留近似力矩标志；输出为安装坐标的绝对或去皮后相对值。详见[参考点和校准约定](../README.md#可选器件处理接口)。
