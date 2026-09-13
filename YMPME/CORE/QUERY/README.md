# 独立射线查询

`YMPME_ENABLE_QUERY=ON` 构建 `YMPME::query`，核心World/Body布局不变。查询不推进物理，不修改碰撞统计，无渲染或传感器依赖。

```c
pme_world_query query = {&world, YMPME_Query_DefaultFilter()};
query.filter.ignoreBody = sensorBodySlot;
pme_ray_provider provider = {YMPME_Query_WorldRaycast, &query};
```

输入 `pme_ray` 的 direction 必须为单位向量，maxDistance为米且大于0。返回最近非负距离表面交点；起点在封闭物体内部时返回出口。未命中返回 `PME_OK, hit=0`，错误与未命中明确区分。

Sphere采用解析交点，Box保持现有轴对齐定义，ConvexMesh先用世界AABB筛选，再转本地坐标逐三角形求交，支持旋转凸网格。查询始终使用实际表面，不受 `world.config.meshPrecision` 的碰撞精度开关影响。场景须保持由合法Body/Mesh接口初始化后的有效状态，扫描期间不得删除/复用或并发改写几何。

过滤包括 `ignoreBody`、非零同装配体忽略、双向category/mask、世界组配对矩阵及 `includeSensors`（默认关闭）。供体ID为World槽位，仅在当前场景生命周期有效；移除/复用Body后旧ID不应继续解释为原对象。

首版逐物体遍历，没有BVH，也没有凹网格、材质透明度或多回波。协议位于 `CORE/YMPME_Ray.h`，LiDAR/Depth只依赖这个函数指针协议；应用可替换为自己的空间索引或不同物理后端，无需修改传感器。射线错误不会部分覆盖查询结果。
