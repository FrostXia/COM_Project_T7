# mpc_controller_adapter

将 GUI/Simulink 的估计位姿接口转换为非线性误差 MPC。输入为 qHat、ref、config、memory；输出为物理角速度、下次求解初值和诊断信息。

默认 MPC 参数来自 `default_config.algorithm`，允许 config.algorithm 覆盖已有字段。通过 `tracking_error_state` 将参考与估计位姿转换为机器人坐标系中的误差，随后调用 `solve_nonlinear_mpc`。预测误差可转换回全局位姿用于诊断。该接口不读取真实位姿。
