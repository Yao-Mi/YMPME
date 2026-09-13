# PME / GRE 共享几何

## 数据与所有权

PME 的 `pme_mesh_view` 描述外部顶点地址、顶点数量/步长、xyz 偏移、坐标缩放，以及连续三角索引或含索引指针的逐面记录。网格按 `YMPME_INDEX_BITS=16/32` 编译，索引类型为 `PME_Index`。使用 `YMPME_ConvexMesh_InitView` 验证并绑定；几何不复制、不分配。传统连续数组用 `YMPME_ConvexMesh_Init`，默认 32 位构建保留原索引参数的类型兼容。

`YMPME_ConvexMesh_Vertex` / `YMPME_ConvexMesh_Index` 统一读取连续与共享布局。SAT、球/网格检测、射线查询、接触流形和 CCD 均使用这条路径。不要再假定共享网格的 `.vertices` / `.indices` 非空；共享数据保存在 `.view`。

顶点单位通过 vertexScale 转换到米；通用视图中的 xyz 必须是三个连续 float。借用缓冲、缩放、拓扑在物理使用期间不可改变，任何几何修改必须停用该网格，重建缓存后再接回。读取期间不得由别的线程写入或释放缓冲。

## GRE 原生绑定

```c
pme_gre_mesh binding = {0};
/* source 为 GRE 创建的单个局部空间凸三角网格。
 * faces/edges 为 PME 轴缓存；renderFaces 为独立 GRE 面片工作缓存。 */
YMPME_GRE_Mesh_Bind(&binding, source, 100.0f,
    faces, faceCapacity, edges, edgeCapacity, renderFaces, renderFaceCapacity);
/* 检查返回值后，把 &binding.mesh 交给 PME 物体/碰撞体。 */
YMPME_GRE_Mesh_PrepareRender(&binding, physicalPosition, physicalRotation);
/* 渲染列表借用 &binding.render。 */
```

GRE 持有 source->pointList 和 polygonList 内的索引数组。PME 直接借用这些地址；显示变换写入 GRE 已分配的 pointList_，不会改写局部数据。面片工作记录保留相同索引指针，更新的是显示法线等工作属性。仍需正常的渲染工作内存与碰撞预计算缓存，零几何复制不等于零额外内存。

PrepareRender 输出普通逐面着色网格，保留基础 UV；高级法线/切线属性路径未接入此适配。不要对 source 调用 LocalToWorld；不要把临时 render 壳交给 YMGRE_Free_Object。源对象的存活由应用负责，绑定不能检测已经被释放的裸指针。

释放顺序：从渲染列表和物理世界移除引用 → YMPME_GRE_Mesh_Unbind → YMGRE_Free_Object(source) → 释放应用持有的缓存。解绑只清空绑定，不释放 GRE 数据。

两库必须使用相同索引位宽；适配头文件对不匹配配置编译报错。PME 不通过 GRE 接口读取数据，适配源仅负责填写通用布局，因此非 GRE 应用也能用同一物理库。

## 几何约束和测试

此共享路径仍使用闭合凸网格碰撞；不会自动变成任意三角汤/凹网格检测。GRE 每个面必须恰好三个顶点，且几何拓扑共享边使用同一索引。带 UV 接缝的重复位置索引、开口、凹面或错误索引由初始化明确拒绝。

16 位支持索引 0～65535（最多 65536 个顶点；GRE 自身上限可能更小），32 位支持更大索引。测试覆盖 uint16 最大值、uint32 超出 16 位范围的索引、错误步长/偏移/空指针、共享与紧凑网格碰撞、射线、接触及 CCD。原生 GRE 测试比对共享地址和原始字节，渲染后再次检查碰撞结果。
