# COM Project T7

项目主要代码分为四个目录：`Trajectory`、`utils`、`controller`、`adapter`。

在 MATLAB 中进入仓库根目录后运行：

```matlab
start_project7
```

启动器会调用 `setup_project7`，不用 `addpath(genpath(...))`。原 `start_trajectory_lab` 保留为兼容入口，但旧磁盘目录已经改名。

GUI 的 Controller 可选择 `demo`、`openloop`、`sf`、`lqr`、`mpc`、`external`。`demo` 仍是接线验证控制器；MPC prediction 可明确选择 `nonlinear` 或 `ltv`，GUI 默认 nonlinear。默认状态估计与位姿测量噪声保留原平台设置；独立误差噪声实验是另一明确标注的运行配置。

```matlab
setup_project7
run_all_tests(false,false) % MATLAB tests; review toolbox SKIP messages
run_all_tests(true,true)   % Also test Simulink and GUI on the local machine
```

独立噪声实验：

```matlab
setup_project7
c=p7utils.defaults;
c.duration=1; c.N=4; c.plot=false; c.save=false;
run_sf_lqr_noise(c)
c.mpc.mode='nonlinear';
run_mpc_noise(c)
```

**独立实验默认仍为 LTV，KF 默认关闭，Qn/Rn 未填写。** 这些设置保留队友的实验版本；需要非线性模式时显式设置。不要把平台位姿 EKF 与误差 KF 的结果当成相同观测实验。

架构、函数归属、重复逻辑及验证限制见 [目录重构与整合说明](docs/目录重构与整合说明.md)。旧使用说明见 [使用说明](docs/使用说明.md)。代码注释使用英文，说明文档使用中文。
