# Project 7 — TrajectoryLab

这是你负责的 MATLAB 交互与 Simulink 仿真环境，正式算法通过独立接口接入。程序注释及界面使用英文，说明文档使用中文。

## 启动

在 MATLAB 中进入本文件所在的 `TrajectoryLab` 文件夹，然后运行：

```matlab
start_trajectory_lab
```

依次选择路径、点击 **Smooth / preview**、检查图形、点击 **Validate path**、点击 **Confirm processed path**、点击 **Run in Simulink**。

手绘线使用橙黄色显示。手绘平滑会在 `Max smoothing shift (m)` 允许的偏移范围内自动加强，默认最多偏离原笔画 0.20 米。最终检查针对平滑后的曲线，而不是原始鼠标抖动。

第一次运行或点击 **Open Simulink model** 时，`build_p7_model` 自动生成 `models/p7_trajectory_harness.slx`。该模型必须由应用或 `p7_run` 配置后运行，不能在没有会话的情况下直接按模型 Run。

## 文件说明

- `docs/使用说明.md`：操作、实验设置、噪声及结果解释。
- `docs/接口与架构.md`：交给算法队友的接口约定。
- `docs/验证记录.md`：已检查内容与尚未完成的运行验证。
- `adapters/p7_external_controller.m`：正式控制器接入口。
- `src/p7_defaults.m`：可配置的演示参数。
- `run_tests.m`：核心、Simulink 一致性及界面测试。
- `run_batch_trials.m`：批量随机初值试验示例。

## 依赖及边界

需要 MATLAB 和 Simulink。基础程序自带演示控制器和 EKF，不要求 MPC、Control System、Image Processing 或 Optimization Toolbox。队友算法如果使用 `fmincon`，另需 Optimization Toolbox。

**demo 是连接测试用的追踪控制器，不是 LQR、MPC 或正式最优控制结果。** 当前路径进度采用局部单调投影，用于平台测试；文档第 13 页的优化进度算法仍由队友实现。

原 Assignment 2 文件夹没有被修改。新模型使用独立编写的固定速度运动学实现，不依赖参考目录的相对路径。

当前运行验证状态见 `docs/验证记录.md`。不要把代码生成成功、静态检查通过或 MATLAB 验证循环运行成功等同于 Simulink 闭环验证通过。
