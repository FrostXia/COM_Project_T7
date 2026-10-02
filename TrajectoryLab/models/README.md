# Simulink 模型

在 MATLAB 中运行 `build_p7_model` 会在此目录创建 `p7_trajectory_harness.slx`。

生成器不会覆盖已经存在的同名文件。修改生成器后如需重新生成，请先把旧模型改名备份并关闭，再运行生成器。

模型采用三个可检查源码的 Level-2 MATLAB S-function，分别处理机器人、测量与估计、参考与控制器。主模型显示闭环连接与上一周期输入延迟。

请通过 `start_trajectory_lab` 或 `p7_run` 运行模型，它们会准备本次会话；直接从模型窗口运行一个没有会话的模型会缺少 `p7SessionKey`。
