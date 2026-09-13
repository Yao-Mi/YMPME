# 电机单摆 Model

`YMPME::pendulum` 复用 RIGID 和 REVOLUTE 关节：一根忽略质量的刚性杆连接球形负载，转轴为世界 Z。配置指定顶部锚点、杆长（锚点到负载质心）、负载质量、球半径和初始角，初始零角竖直向下。角度允许 [-pi,pi]；初始速度为零。

启用 `YMPME_MODEL_PENDULUM=ON`，自动包含 JOINT/RIGID，不包含 SENSOR、GRE 或其他 Model。状态内嵌一个刚体、一个关节，全部内存由调用者持有，无内部指针和堆分配。

```c
pme_pendulum model;
pme_pendulum_config config = YMPME_Pendulum_DefaultConfig();
config.initialAngle = .6f;
YMPME_Pendulum_Init(&model, &config);
YMPME_Pendulum_Step(&model, YMPME_Vec3(0,-9.81f,0), 1.f/240);
```

默认自由摆。通过 `model.joint` 设置电机、限位或释放，通过 `model.body` 施加外力。`Pendulum_Step` 统一积分并更新 `model.time`。步进之后读取 `joint.feedback` 与 `body.feedback`，无需依赖 GRE。

GRE 示例 [joint_lab](../../../../project_Demo/joint_lab/README.md) 在应用侧实现 PD + 重力补偿，并接 Encoder；模型自身不强制控制策略或传感器。小角度周期按 `2*pi*sqrt((length^2 + 0.4*radius^2)/(g*length))` 验证，静态电机力矩按 `mass*g*length*sin(angle)` 验证。无杆弯曲、绳子松弛或形状碰撞。
