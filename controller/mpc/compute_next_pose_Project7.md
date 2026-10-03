# compute_next_pose_Project7.m 逐段说明

**作用：**恢复原 ZIP 的职责及六参数接口，推进机器人全局实际位姿。

**调用关系：**launch_Project7_MPC → compute_next_pose_Project7 → p7utils.pose_step / wrap。

**边界：**不是误差预测函数；独立无噪声入口固定 100 小步。Q5 噪声包默认同样 100 小步，GUI 使用平台自带物理模型。

代码中的 %% 是主要逻辑分段；下面按执行顺序解释。短辅助函数没有强行拆成多段。完整源码附在末尾，便于逐项核对。

### 第 1 段：实现与原理

输入 theta,x,y,v,omega,dt；依原接口返回 theta_next,x_next,y_next，注意输出顺序。

### 第 2 段：实现与原理

调用 Q5 heading-first 100 小步积分，最终只将航向 wrap；这与原 ZIP 单步 Euler 是明确的数值积分差异。

## 验证状态

MATLAB 语法树解析通过；相关数学使用 Python/SciPy 作了独立核对。没有在交付环境执行 MATLAB 或 Simulink，具体运行检查见包内 VALIDATION.md。对原平台复制函数，本次只做源码、调用和语法核对，未将原作者的静态记录当作本次运行结果。

## 对应源码

~~~matlab
function [theta_next,x_next,y_next]=compute_next_pose_Project7(theta,x,y,v,omega,dt)
% COMPUTE_NEXT_POSE_PROJECT7 Advance physical pose; same API as original ZIP.
% Use the Q5 heading-first 100-substep plant, also used in the noise packages.
q=p7utils.pose_step([x;y;theta],v,omega,dt,100);
theta_next=wrap(q(3));
x_next=q(1); y_next=q(2);
end
~~~
