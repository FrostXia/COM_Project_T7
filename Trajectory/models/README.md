# Simulink 模型

在 MATLAB 中运行 `build_simulation_model` 会在此目录创建 `trajectory_simulation.slx`。

生成器不会覆盖已经存在的同名文件。修改生成器后如需重新生成，请先把旧模型改名备份并关闭，再运行生成器。

模型采用三个可检查源码的 Level-2 MATLAB S-function，分别处理机器人、测量与估计、参考与控制器。主模型显示闭环连接与上一周期输入延迟。

请通过 `start_app` 或 `run_simulation` 运行模型，它们会创建本次仿真会话，并将数值会话编号传入各 S-function；直接运行模型不能代替应用中的实验初始化。
