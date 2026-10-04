# Trajectory

界面与运行调度目录。共享函数位于 `utils`，控制器位于 `controller`，接口转换位于 `adapter`。

从仓库根目录运行 `start_app`。`start_trajectory_lab` 仅作为旧名称的启动别名，最终也启动原生 App Designer 界面。

- `designer/`：主界面、路径编辑器和噪声窗口的 `.mlapp`。
- `src/TrajectoryAppController.m`：共享交互、路径处理、绘图、运行与导出逻辑，不创建控件。
- `src/bind_designer_views.m`：按 Tag 连接控件与行为。
- `src/SimulationSession.m`：单次运行的真实状态、估计、噪声和控制会话。
- `src/run_simulation.m`：Simulink 调度，以及测试使用的 MATLAB 验证循环。
- `src/sfun_*.m`：Simulink S-functions。
- `models/`：由 `build_simulation_model` 生成的模型。

独立实验及批量实验入口已删除。当前操作见 [三步界面使用说明](../docs/三步界面使用说明.md)。
