# 通用三维刚体动力学

`YMPME::rigid` 提供质心平移、四元数转动、主轴惯量、世界坐标力/力矩及冲量。单独启用 `YMPME_ENABLE_RIGID=ON`，无需 JOINT、MODEL、SENSOR、GRE 或 SDL；默认关闭。`pme_rigid_body` 是独立类型，旧 `pme_body/pme_world` 的布局和行为不变。

`YMPME_Rigid_Init` 接收质量、主轴惯量、质心位置和单位四元数。质量为 0 表示固定体，不支持赋予速度的运动学平台；动态体的三个主惯量必须为正。本体坐标系须经过质心、与惯量主轴重合。`YMPME_Rigid_BoxInertia` 的尺寸是盒子全尺寸，单位 m。`mass/inverseMass/inertia/inverseInertia` 是初始化后的只读参数；改变质量属性应重新初始化。

- `AddForceAt`：世界坐标力和施力点，自动计算关于质心的力矩。
- `AddTorque`：世界坐标纯力矩。
- `Impulse`：线冲量与关于质心的角冲量，用于自定义求解器。
- `Step`：自由体一个固定子步。重力单独传入；用户加载步末清零。
- `Begin/End`：供 JOINT 或自定义求解器按“加载速度 → 约束冲量 → 积分姿态”调用，不应再对同一物体调用 `Step`。

内部使用半隐式平移、指数映射姿态更新和显式陀螺项。建议 dt=1/240 s，允许 (0,0.05] s；高角速度、较大惯量比和高频控制需要减小步长。没有休眠、CCD、接触流形或转动接触求解，也不自动接入旧 World 的碰撞。旧 AIRCRAFT 暂时继续使用自己的积分器。

`body.feedback` 与现有 `pme_feedback_state` 格式一致：位置、速度、角速度、线/角加速度全部是世界坐标；加速度为整个子步的速度差分，包括重力及约束。初始化时导数位无效，首步后有效。关节驱动时由 `YMPME_Joint_Step` 统一更新，IMU 可直接采样该状态。

全部状态由调用者持有，物理路径不申请内存。四元数矩阵、乘法、Log 等数学函数要求有效单位四元数，可用 `YMPME_Quaternion_IsValid` 检查。普通 `Step` 和 JOINT 的 `Step` 会先检查状态与参数；分阶段低层接口由调用者负责先验证。仿真时间由调用者提供，不使用渲染帧时间。

## 接触共同求解

启用可选 [DYNAMICS](../../DYNAMICS/README.md) 后，可将本模块的刚体与 JOINT、球/盒子/凸 Mesh/平面接触统一步进，产生角冲量与三维摩擦。上述自由体模块自身仍不负责碰撞，也没有修改旧 World。
