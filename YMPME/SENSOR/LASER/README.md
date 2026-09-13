# 单点激光测距

`YMPME_SENSOR_LASER=ON` 提供独立的 `YMPME::laser`。默认关闭，依赖公共 SENSOR / 射线协议，既不要求整圈 LIDAR，也不要求 World 查询模块、GRE、DYNAMICS 或 skin。应用传入任意符合 `pme_ray_provider` 协议的场景查询器，每次测量只查询一条最近命中射线。

`YMPME_Laser_DefaultDesc()` 默认沿安装坐标 +Z，量程 0.02～20 m，偏差/噪声为零，分辨率 0.001 m。安装偏移和旋转通过 `mount` 设置，随主体位置/姿态改变。`resolution=0` 关闭量化；`noise` 为标准差指定的均匀噪声，与现有传感器约定一致。数值单位均为米。

```c
pme_laser_desc laser = YMPME_Laser_DefaultDesc();
pme_laser_sample sample;
uint32_t seed = 123; /* 每个实例持有独立种子 */
pme_result result = YMPME_Laser_Measure(&laser, &bodyState, &stamp,
                                     &rayProvider, &seed, &sample);
if (result == PME_OK && sample.valid) {
    float distance_m = sample.distance;
    /* 控制器消费 distance_m。 */
}
```

采样率和延迟元数据使用现有 `pme_sensor_clock`；调用者按仿真时间调度，不含隐式时钟、队列或渲染依赖。调用成功且没有有效距离也会更新样本：

| status | 含义 |
|---|---|
| PME_LASER_VALID | 距离有效 |
| PME_LASER_NO_RETURN | 量程内无回波；也可能目标在量程以外 |
| PME_LASER_BLIND | 最近表面位于最小量程内，阻挡后方目标 |
| PME_LASER_OUT_OF_RANGE | 偏差/噪声/量化后的读数越出量程 |

后三种 `valid=0`、`distance=NAN`，不能继续沿用旧距离。`stamp.valid` 表示这次采样成功，和距离有效位分开。参数错误或提供者失败返回错误码，并保持输出和 RNG 种子不变；调用者必须检查返回值。

`origin/direction/rayEnd/hit/body` 保存未经噪声处理的几何追踪结果，供可选 GRE 适配显示。命中盲区仍截断激光，未命中则终止于最大量程。读取这些字段不会再查询一次场景。控制器使用测量值 `distance`，无需依赖调试字段。

当前为瞬时理想单回波几何模型，没有光斑尺寸、表面反射强度、透明物体折射、多径或真实飞行时间电路。物体仅需参与场景碰撞查询，无须安装 skin。

独立构建：

```bash
cmake -S . -B build/laser-core -DYMPME_BUILD_DEMOS=OFF -DYMPME_SENSOR_LASER=ON
cmake --build build/laser-core --target test_sensor_laser
ctest --test-dir build/laser-core -R ympme_sensor_laser --output-on-failure
```

传感器实验台可启用相同开关，J 切换单点激光的关闭/点/线/点＋线显示。单元测试覆盖一次查询、安装变换、量化、边界、盲区遮挡、无回波、噪声可重复与失败回滚。
