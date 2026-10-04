# predict_tracking_error

使用非线性误差方程预测一步，采用 RK4，子步数由 predictionSubsteps 指定。

误差为参考减实际，表达在机器人坐标系内；控制修正量 u = omega - omegaReference。每个预测区间使用对应参考角速度 wr 和参考线速度 vr。全局真实机器人仍由平台 integrate_robot_motion 推进，预测器不访问真实状态。
