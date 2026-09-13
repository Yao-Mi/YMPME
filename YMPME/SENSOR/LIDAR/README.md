# 2D 360° 激光雷达

启用 `YMPME_SENSOR_LIDAR=ON`，链接 `YMPME::lidar`。默认示例为水平 XZ 平面 360 束、1° 一束、20 Hz 单圈扫描；束数由调用方指定。`YMPME_Lidar_Scan360` 在初始化时预计算方向，`i` 束角度为 `2*pi*i/count`，不重复 360° 端点。+Z 为前、+X 为右、+Y 为上；完整单圈数据一次生成。

```c
pme_vec3 beams[360];
pme_lidar_point points[360];
pme_lidar_frame frame = {0};
frame.points = points;
frame.capacity = 360;
YMPME_Lidar_Scan360(beams, 360, 360);
pme_lidar_desc desc = {YMPME_Sensor_IdentityMount(), .05f, 12, 0, 0};
/* state/stamp为同一个仿真时刻；provider由应用提供，seed为实例私有。 */
YMPME_Lidar_Measure(&desc, beams, 360, &state, &stamp, &provider, &seed, &frame);
```

以上函数都返回 `pme_result`，调用者应检查返回值。每束返回 `range`（米）、`point`（安装坐标下的点，2D时y=0）、`body`（提供者对象ID）和 `valid`。未命中或落在盲区：`valid=0, range=NAN, point=(0,0,0), body=SIZE_MAX`，仍保留原束下标。世界点为 `frame.origin + frame.rotation * point`。

近处物体即使落在最小量程内，仍然遮挡后方物体；不会透过盲区内的障碍选取第二个回波。可指定距离偏置与均匀噪声（noise为标准差，米），用独立种子实现可重复采样。没有反射强度、透明材质、多回波和旋转扫描运动畸变；这是单时刻的理想几何单回波扫描。

射线提供者仅需实现 [pme_ray_provider](../../CORE/YMPME_Ray.h)，传感器不依赖 World、GRE 或 SDL。要查询现有物理世界，额外启用 `YMPME_ENABLE_QUERY` 并使用 [World Query](../../CORE/QUERY/README.md)；可过滤自身、装配体、类别和组。未实现 BVH，复杂场景可换成自带加速结构的提供者。

输出和方向缓存由调用者持有，不得重叠，不分配内存。参数/容量错误在开始采样前返回，保留原输出；提供者中途错误会使 `stamp.valid=0,count=0`，缓存可能已部分写入，随机种子不提交。扫描过程中场景和安装姿态应保持不变。
