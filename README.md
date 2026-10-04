# COM Project T7

项目主要代码分为四个目录：`Trajectory`、`utils`、`controller`、`adapter`。

在 MATLAB 中进入仓库根目录后运行：

```matlab
start_app
```

界面仅使用 `Trajectory/designer` 下三个原生 `.mlapp`，步骤为 Trajectory Setup、Model Selection、Run Simulation。`TrajectoryAppController.m` 负责行为，不再创建备用界面。

运行 `edit_interface` 可视化编辑主界面；参数 `'path'`、`'noise'` 分别打开路径编辑器和噪声窗口。见 [App Designer 界面编辑说明](docs/AppDesigner界面编辑说明.md)。

提供 Open Loop、State Feedback、LQR、非线性 MPC。Nominal Omega R 默认按路径自动计算，State Feedback/LQR 可在 Step 2 手动修改，内部保存为 `nominalOverride`（rad/s）。重新处理路径后恢复自动值。Q/R 和 Qn/Rn 仍仅在代码中配置。

Desired Duration 只读，等于处理后路径长度 / Desired Trajectory Speed。仿真到时结束，不增加终点追赶时间。参考与机器人共用线速度。

随机出生统一调用 `sample_initial_pose`：相对起点，X/Y 分别在对应路径跨度的 ±10% 内均匀取样；零跨度轴借用另一轴跨度；朝向在 [-pi,pi) 上均匀取样。Step 2 仍可手动调整位姿。

Step 3 的 Noise 弹窗独立设置 Measurement / Control / State Noise。控制噪声默认开启，标准差 0.1 rad/s；加噪后的实际角速度仍受 Max Omega 限幅。结果图只显示真实机器人轨迹的红色虚线。见 [三步界面使用说明](docs/三步界面使用说明.md)。

独立实验、批量扫描、演示控制器、LTV MPC、旧版界面创建代码及历史说明文档已移除。测试用 MATLAB 仿真循环保留，用于和 Simulink 比较。

```matlab
setup_paths
run_all_tests(false,false) % Algorithms and MATLAB verification loop
run_all_tests(true,true)   % Also verify Simulink and native UI
```

导出当前实验会生成同时间戳的 `trajectory_时间戳.csv`（完整平滑路线，仅 XY，可重新导入）和 `evaluation_时间戳.csv`（配置与评价指标，仅 Metric、Value、Unit 三列），不合并实验。已有实验输出不会因代码清理而删除。
