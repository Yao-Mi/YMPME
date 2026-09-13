# 可裁剪的曲面皮肤接触模型

`YMPME_MODEL_SKIN=ON`，链接 `YMPME::skin_contact`。默认关闭，只依赖CORE，不依赖SKE、GRE、传感器、关节或堆分配。`CORE/YMPME_Surface.h` 定义世界米制的任意曲面采样单元：位置、朝外单位法线、速度和面积。传入数据可来自骨骼蒙皮、刚体曲面或自己的网格实现。

`YMPME_SkinContact_Sphere` 是球形压头与采样曲面的弹性地基近似，每单元独立计算：先用球AABB排除，再判断采样点是否在球内且法线朝向球心；穿入量为半径减球心距离。压力 `max(0, stiffness*depth - damping*dot(probeVelocity-skinVelocity, normal))`，受maxPressure上限限制；压缩力沿皮肤负法线。切向黏性阻力沿相对滑动方向，受friction乘法向力上限限制，无静摩擦粘着历史。反作用力相反，力矩关于球心。刚度单位Pa/m，阻尼与切向阻尼Pa·s/m，面积m²，结果N/Nm。压力封顶设置saturated。

这里不是精确三角网格碰撞、连续碰撞检测、完整皮肤组织/FEM，也不会自己修改顶点。接触面积取命中的采样单元面积，网格/采样分辨率影响结果；很小的探头可能漏过稀疏点之间的面。背面不会吸附，离开表面载荷为零。法向采用局部皮肤法线，因此球面上的反力可能有非零力矩；这也是此近似的边界。需要严格球面无摩擦接触力时应换真实接触流形求解器。

应用决定载荷如何作用到运动系统；模型不调用Joint或World。低模手演示通过蒙皮权重分配力到PME指节，并把每份力施加在同一世界采样点，从而保持总力/总力矩。这是应用级耦合近似，不代表严格的LBS雅可比或软体动力学。

数组由调用者持有，输入与输出不得重叠，包含错误输入/容量不足/计算溢出的调用不提交输出。函数先预检并累计，再写各单元结果。所有查询均为当前时刻瞬时接触，应用以固定物理步长采样后将该力作为该步外载荷。

## 自由转动球

`YMPME_SkinContact_SphereRigid` 接收世界角速度，使用径向接触法线与 `v + omega × r` 的相对接触速度。正常压力不产生球心转矩；切向摩擦产生反作用力矩。输出仍是皮肤上的每个 patch 力和关于球心的反作用 wrench，调用者负责刚体积分一次。原 Sphere API（皮肤法线按压）和全部结构布局不变。两者均为表面点采样近似，不是三角面或连续碰撞。

## 区域自体接触

`YMPME_SkinSelfContact.h` 的 `YMPME_SkinSelfContact_Solve` 接收 `pme_skin_face[]`（世界三角面、patch、区域号）、材料、虚拟接触层厚度、搜索范围，以及最多 32 个区域的双向过滤矩阵。`sourceRegions` 与 `targetRegions` 必须互斥；两侧 `allow[]` 均启用才碰撞。相邻部位和同一连续皮肤表面的排除由应用配置，函数不猜测骨骼或拓扑。

依次执行区域 AABB、采样点与三角形 AABB、最近点精筛。每个源三角形的中心及三个顶点各占面积的 1/4，每个点只选一个目标面。虚拟接触层提供压缩深度，阻尼和黏性切向摩擦沿用材料单位与库仑上限。此算法是有厚度的表面采样近似，未实现完整面—面、边—边或 CCD。背离的法线、三角边界外超出厚度的采样不产生载荷；瞬移穿过搜索范围会漏检。

调用者提供 `forces[]`、`moments[]` 和 `pme_skin_self_target targets[]`（目标三角形的索引和 AABB 缓存） 工作数组，容量均至少为面数，互不重叠。`moments[i]` 关于该面中心；汇总世界力矩时使用 `cross(faceCenter-reference, forces[i]) + moments[i]`。双方力施加于相同接触点，因此统一参考点的合力和合力矩抵消。`sourceWrench/targetWrench` 关于调用时给定的 reference，`maxDepth` 用于应用侧姿态约束，`saturated` 表示压力封顶。函数不分配内存，不修改输入几何。

默认过滤矩阵全部关闭；例如源区域 1 与目标区域 2，设置 `sourceRegions=1u<<1`、`targetRegions=1u<<2`、`allow[1]=1u<<2`、`allow[2]=1u<<1`。人体示例的 IK、绑定权重分区和渲染仅在 demo 层；核心模型不依赖 SKE、GRE、JOINT 或 SENSOR。两类接触实现在独立源文件内，静态链接的旧球形探头应用不引用自体接触对象。


## 平面接触

`YMPME_SkinPlane.h` 的 `YMPME_SkinContact_Plane` 接收世界坐标平面单位法线、offset、平面速度、载荷参考点和柔顺层厚度 shell。实体位于 `dot(normal, position) < offset` 一侧；朝向地面的面片中心进入 shell 后，按面积在平面上的投影生成压力与摩擦。正常力沿平面法线向外，切向黏性摩擦抵抗相对滑动并受 `mu * normalForce` 限制；分离时不产生吸力。

输出 `forceOnSkin[]` 为逐面片世界力，结果中的 `probeWrench` 在这个 API 中表示**平面反作用载荷**，力矩关于 `plane.reference`，与同一参考点下的皮肤载荷抵消。使用现有结果类型以复用采集路径；不表示存在球形探头。全部输入预检后写入，无堆分配。此模型不积分骨骼、不做连续碰撞或精确三角形裁剪，仍是面片中心采样近似。与其他 SKIN 源文件一起受 `YMPME_MODEL_SKIN` 裁剪，没有 GRE/SKE 依赖。示例见 `project_Demo/human_ground`。
