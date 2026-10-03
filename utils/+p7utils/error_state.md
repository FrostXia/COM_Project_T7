# error_state.m 逐段说明

**作用：**把全局位姿差转换为本体误差，并处理航向周期。

**调用关系：**仿真真值误差、TrajectoryLab 估计位姿适配都调用此函数。

**边界：**只负责坐标变换；不给真值误差添加噪声。

代码中的 %% 是主要逻辑分段；下面按执行顺序解释。短辅助函数没有强行拆成多段。完整源码附在末尾，便于逐项核对。

### 第 1 段：实现与原理

位置先作参考减实际，再由机器人当前航向旋转。

### 第 2 段：实现与原理

atan2(sin(angle),cos(angle)) 将航向误差映射至主值分支。

## 验证状态

MATLAB 语法树解析通过；相关数学使用 Python/SciPy 作了独立核对。没有在交付环境执行 MATLAB 或 Simulink，具体运行检查见包内 VALIDATION.md。对原平台复制函数，本次只做源码、调用和语法核对，未将原作者的静态记录当作本次运行结果。

## 对应源码

~~~matlab
function e=error_state(q,qr)
% ERROR_STATE Reference-minus-actual error in the robot body frame.
d=qr(1:2)-q(1:2);
th=q(3);
e=[cos(th)*d(1)+sin(th)*d(2);-sin(th)*d(1)+cos(th)*d(2); ...
    atan2(sin(qr(3)-th),cos(qr(3)-th))];
end
~~~
