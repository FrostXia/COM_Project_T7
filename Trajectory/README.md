# Trajectory

这是原 TrajectoryLab 改名后的界面与运行调度目录。共享函数已经移到上一级 `utils`，控制器在 `controller`，接口转换在 `adapter`。

从仓库根目录启动 `start_project7`，或从本目录启动 `start_trajectory`。兼容入口 `start_trajectory_lab` 会调用同一个新入口。

- `src/P7App.m`：界面、绘图、路径确认。
- `src/P7Session.m`：单次平台运行会话。
- `src/p7_run.m`：MATLAB / Simulink 调度。
- `src/p7_sf_*.m`：Simulink S-functions。
- `experiments/`：独立 MATLAB 实验、扫描和原 MPC launch。
- `models/`：本机由 build_p7_model 生成模型。

详见仓库根目录 README 和 docs/目录重构与整合说明.md。
