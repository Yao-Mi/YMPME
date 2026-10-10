# 实体有限元静力分析（TET4）

`YMPME_ENABLE_SOLID_STATICS=ON` 启用 `YMPME::solid_statics`，宏为
`PME_ENABLE_SOLID_STATICS=1`。默认关闭，独立于二维/三维杆梁静力、GRE、SKE、
DYNAMICS 和传感器。源代码位于 `YMPME/STATICS/YMPME_SolidStatics.c`。

## 模型与结果

四节点线性四面体，三维小应变、均匀各向同性线弹性材料。每节点 XYZ 三个位移
自由度，可施加节点力和指定位移；不含转角自由度。模型输入坐标、四面体连接、
E（Pa）与泊松比，要求 `-1 < nu < 0.49`，不支持近不可压缩材料。
支持单个连通、无退化、相容的实体网格；四面体顶点正反朝向都可。
调用者负责网格无重叠、无裂缝及材料/载荷的单位一致性。

每节点输出位移（m）、反力（N）。每个四面体输出：

- 六个应变及应力分量：xx、yy、zz、xy、yz、zx；剪应变采用张量分量 εxy，而非工程剪应变 γxy。
- von Mises 应力（Pa），应变能密度（J/m³），参考体积（m³）。
- 四面体内部应变和应力为常量，不进行节点平均或人为平滑。

约束是双向位移约束，允许抗拔，不是单向接触。模型不含自动重力、分布载荷
转换、接触、塑性、破坏、屈曲、疲劳或动态响应。面载荷需调用者按面形函数
分配到节点；窗口例子已按三角面面积把载荷一致分配为节点力。

## 求解与内存

不组装稠密总体矩阵，使用按单元应用刚度的预条件共轭梯度（PCG）方法和对角
预条件。内存 O(节点数+单元数)，库内无堆分配。默认相对容差 1e-8、绝对残差
容差 1e-9 N、最大 3000 次迭代，可通过 `pme_solid_options` 修改。
`pme_solid_workspace_doubles(N,T)` 返回 `21*N+13*T` 个 double 的工作区大小。
模型、输出、工作区不能重叠。只有返回 `PME_SOLID_OK` 时结果有效。

会拒绝退化单元、孤立节点、不连通网格及未约束的整体刚体模态。只在顶点或边
相连的铰链式体集合不属于支持的单一相容实体；调用者必须消除内部机构。
报告包含实际残差、相对残差、迭代数和总应变能。迭代上限或数值失败不作为
有效结果输出。无自由载荷时相对残差定义为零。

## 验证和使用

```sh
cmake -S . -B build/solid -DYMPME_ENABLE_SOLID_STATICS=ON
cmake --build build/solid --target test_solid_statics demo_solid_statics -j
ctest --test-dir build/solid -R ympme_solid_statics --output-on-failure
./build/solid/demo_solid_statics
```

验证覆盖包含内部自由节点的三维仿射位移场、全部剪应变分量、单轴压缩的泊松
收缩和应力解析解、翻转单元朝向、能量一致性、力/力矩平衡、网格加密对位移
的影响，以及欠约束、容量和未收敛等失败情况。
通过 SDK 的 `solid_statics` 组件导出，消费端链接 `YMPME::solid_statics`。

## 如何用于加强或减重判断

先确定全部实际载荷工况和支承，观察高应力/高应变能区域，结合材料许用值及
失效模式决定加强、圆角或载荷路径调整。低应力区域只是后续几何试验的候选，
不能凭一种载荷下的蓝色区域直接判定可删除。特别是 von Mises 对静水压力不敏感，
低等效应力不等于所有应力分量都低。

线性四面体在弯曲问题中可能偏硬，需网格加密对照。至少比较位移、能量和关注
位置的应力，而不只是求解残差。尖角、刚性夹持边缘和载荷边界附近的峰值可能
具有网格依赖性或奇异性；不能把云图最大值直接当成已经收敛的强度结论。
修改孔洞/厚度后需要重新划分网格与求解，再检查其他工况、刚度、稳定性和疲劳等。
当前模块不是拓扑优化器，也不会自动建议安全删除体积。

参考：
[SolidWorks 四面体单元](https://help.solidworks.com/2023/english/SolidWorks/cworks/c_solid_mesh.htm)、
[单元应力与节点应力](https://help.solidworks.com/2026/english/SolidWorks/cworks/c_Node_Values_versus_Element_Values.htm)、
[应力奇异性与网格收敛](https://blogs.solidworks.com/products/solidworks/what-is-a-stress-singularity-in-solidworks-simulation/)、
[COMSOL 应力计算](https://doc.comsol.com/6.3/doc/com.comsol.help.sme/sme_ug_theory.06.159.html)。
