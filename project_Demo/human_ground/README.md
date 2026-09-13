# 人体地面运动与全身皮肤观测

直接运行 `build/project_Demo/human_ground/demo_human_ground`。GRE 窗口左侧显示全身、地面网格、左右脚向上的接触力箭头和载荷读数；右侧默认一张 64×64 全身拓扑网格图（窗口按 8× 放大到 512×512），头、躯干、手、腿、脚共 3584 个固定面片。

| 操作 | 功能 |
|---|---|
| 1 / 2 / 3 / 4 | 向前行走 / 站立换重心 / 下蹲 / 双脚离地 |
| 空格 | 暂停 |
| 方向键、3D 区滚轮 | 旋转相机、缩放；相机随行走位置移动 |
| W / B / H | 线框、骨骼、三维受力颜色 |
| V / Y | 拓扑网格 / 三角图；网格按身体部位配色，方便检查聚集 |
| T / C | RGB、力模长、压力；显示量程 |
| 点击 3D 或 UV | 查看面片 Fn/Fu/Fv（N） |
| L / R / U | 在同一张全身图中放大左脚 / 右脚 / 恢复全图 |
| UV 滚轮 | 局部放大；观测数据仍是完整固定全图 |
| Esc | 退出 |

## 动作与接触

源模型没有动画片段。demo 用两段腿部 IK 生成抬脚、迈步和下蹲，保持脚的绑定朝向，手臂跟随摆动。步周期 2 秒，摆动抬脚最高约 7 cm，前进速度 0.08 m/s。站立模式通过小幅脚部压缩变化观察双脚载荷变化；离地模式将整个人抬起用于验证受力归零。

接触复用纯 PME `YMPME_SkinContact_Plane`，对全身面片查询地面，不按动画阶段填造力值。向地面的面片中心进入 6 mm 柔顺层后，按实际面积在地面上的投影计算弹性压力、法向阻尼和受库仑上限限制的切向黏性摩擦。全部单位为 SI，120 Hz。脚部读数为世界向上的合力，皮肤图 R 为各面片外法线方向的压缩力，两者可因面片倾斜而不同。离地没有接触则 RGB 回到零载荷。

这是**规定骨骼运动的接触传感演示**：尚不是由重力、关节力矩及平衡控制器驱动的动力学行走，载荷不强制等于某个体重。地面模型是面片中心采样近似，尚无三角裁剪、软体变形或连续碰撞。该模块随 `YMPME_MODEL_SKIN` 裁剪，不依赖 GRE/SKE；两者只在 demo 层对接。

## 固定观测接口

与 `human_skin` 共用 `../human_skin/observation.h`。绑定姿态生成一次全身 UV，所有三角大小相同，布局不随动作改变。512×512 每个有效像素对应固定面片 ID；ID 对应 `rig.triangle/patches/forces` 的下标。初始化检查全部 3584 面片都有观测像素。

- `HumanSkin_ObservationMap(&rig)`：固定 `uint16_t[512*512]` 面片图。65535 表示背景，可直接派生有效像素 mask。
- `HumanSkin_Observe(...)`：调用者提供 RGB 字节缓冲、逐面片 `pme_vec3` 浮点力缓冲及时间戳输出；不推进仿真，不分配内存。浮点向量顺序为 `(Fu,Fv,Fn)` N。
- RGB 按 **R=Fn、G=Fu、B=Fv** 编码；默认 R 为 0～20 N，G/B 为 ±10 N，所有部位共用。量程在 `contactView` 中显式配置，保持固定，独立于 C 显示量程。切向中值 128 表示零，饱和值应结合浮点通道使用。
- 有效皮肤无接触 `(0,128,128)`，背景 `(0,0,0)`；图像为左上原点、逐行排列。没有网格线、光照、选中框或相机数据。

离线导出（目标目录需先存在）：

```sh
build/project_Demo/human_ground/demo_human_ground --export /tmp/skin-observation
```

程序推进到 3.5 秒，写出 `skin_rgb.ppm`、16 位大端 PGM `skin_face_ids.pgm`、逐面片浮点 `skin_forces.csv` 和含 mesh ID、时间、分辨率、量程的 `skin_metadata.json`。导出仅由显式命令触发。模型或布局算法变化时应重新保存面片映射，不能跨版本假定像素 ID 不变。

## 构建

从 YMPME 根目录：

```sh
cmake -S project_Demo/human_skin -B build/human \
  -DCMAKE_BUILD_TYPE=Release \
  -DHUMAN_DEMO_OUTPUT_ROOT="$PWD/build/project_Demo"
cmake --build build/human -j8
ctest --test-dir build/human --output-on-failure
```

人体两项 demo 共用六槽骨骼编译配置，与其他四槽 demo 隔离。直接运行可执行文件；不安装桌面应用，不创建启动脚本。


全身小图已使用 PME 轮廓装箱，将小图块插入大图块空洞；保持所有邻接和统一三角尺寸。面积利用率约 39.7%（上一版约 25.6%）。只在启动时排布，动作期间映射固定。此次布局升级改变像素位置；已有智能体数据应重新导出对应的面片 ID 图。


## 拓扑网格观测（默认）

每个面片只占一个逻辑格子，3584 个面片恰好出现各一次；64×64 中其余 512 格为空。窗口每格放大为 8×8 像素，原生智能体数据仍是 **64×64、一像素一面片**。网格由共边拓扑扩散和独占格子交换生成，生成后固定。Y 的部位色仅用于检查，骨骼分区不参与排布计算。V 保留三角图对照。

- `HumanSkin_GridObservationMap`：`uint16_t[4096]`，65535 为空，其余每个面片 ID 恰好出现一次。
- `HumanSkin_ObserveGrid`：原生 RGB 缓冲 64×64×3 字节，另输出逐面片浮点力和仿真时间戳；颜色量程与原 `Observe` 一致，不随 V/Y/C、相机或缩放改变。
- `HumanSkin_GridNeighbours`：每面片 3 个真实共边邻居，边界 -1。二维相邻格子不一定在皮肤上相邻。
- `contactView.faceCells`：面片→格子的反向查找；结果含邻接距离指标及固定映射 hash。

`--export` 现在额外写出 `skin_grid_rgb.ppm`、`skin_grid_face_ids.pgm`、`skin_grid_metadata.json`、`skin_grid_adjacency.csv`；元数据记录 64×64、独占格子、映射 hash、mesh ID、时间和量程。之前的 512×512 三角 atlas 导出保持原格式，便于对照，不应与新网格 ID 图混用。

本次人体布局的共边平均距离约 1.86 格，65.9% 在 2 格以内。保近邻是近似优化，不能保证每条边都落在相邻格；原始拓扑表与精确面片数据仍可用于智能体。完整方法见 [拓扑网格](../../YMPME/MODEL/SURFACE_ATLAS/TOPOLOGY_GRID.md)。

### 栅格拾取测试

点击 **PICK TEST OFF** 开启测试，再点击右侧栅格，左侧人体对应的唯一三角面片会以亮黄色描边和填充线高亮，并跟随动画。采用透视叠加，因此背面或被遮挡的面片也能定位；此颜色不代表受力。信息栏显示面片编号和力。点空格清除选中，再点按钮关闭高亮。仅影响显示，不改变传感器 RGB、力数据或 face/cell 映射。

### 64×64 面积缓存

`HumanSkin_GridAreas(&rig)` 返回与 `HumanSkin_GridObservationMap` 完全对齐的 `const float[4096]` 视图（行优先、左上原点、m²）。空格为 0，非空格为初始化姿态的整片三角面积。数组由 rig 持有，初始化时复用已有面片面积写入一次，之后不随蒙皮变形更新；读取不分配内存、不重新计算几何。它占用 16 KiB（32 位 float）；同一实例中面积和 ID 映射固定，可保存一次后重复使用。

```c
const float *area = HumanSkin_GridAreas(&rig);
const uint16_t *ids = HumanSkin_GridObservationMap(&rig);
size_t cell = y * HUMAN_GRID_OBSERVATION_SIZE + x;
float meanPressurePa = area[cell] > 0 ? local[ids[cell]].z / area[cell] : 0;
```

示例中的 `local` 来自同一帧 `HumanSkin_ObserveGrid`，其 z 为 Fn（N）。这里得到以初始参考面积为基准的整片平均压强，不是部分接触区域的局部压强。按当前约定忽略蒙皮导致的面积变化；后续每帧只更新力即可。

导出新增 `skin_grid_area_m2.csv`：无表头、64 行×64 列，以 9 位有效数字保存 float，坐标顺序与 ID/RGB 图一致。`skin_grid_metadata.json` 记录面积文件、单位、定义和空格值，时间沿用同一观测的 `time_s`。
