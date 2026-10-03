# kf_error.m 逐段说明

**作用：**可选的误差状态时变 KF，与直接带噪误差反馈明确区分。

**调用关系：**由 simulate_q5 调用；独立控制器运行有独立 m。

**边界：**线性近似 KF，仅在对应模型范围内合理。不是原 TrajectoryLab 全局位姿 EKF。

代码中的 %% 是主要逻辑分段；下面按执行顺序解释。短辅助函数没有强行拆成多段。完整源码附在末尾，便于逐项核对。

### 第 1 段：实现与原理

none 模式直接输出测量、不访问 Qn/Rn；kf 模式在任何计算前拒绝空协方差。

### 第 2 段：实现与原理

KF 用上一周期实际限幅后的指令修正量预测；未知执行噪声不泄漏给估计器。

### 第 3 段：实现与原理

将标量角速度扰动协方差通过 Bd 映射到三维离散状态协方差。

### 第 4 段：实现与原理

三个误差均测量，C=I；航向创新 wrap，Joseph 形式更新协方差。

## 验证状态

MATLAB 语法树解析通过；相关数学使用 Python/SciPy 作了独立核对。没有在交付环境执行 MATLAB 或 Simulink，具体运行检查见包内 VALIDATION.md。对原平台复制函数，本次只做源码、调用和语法核对，未将原作者的静态记录当作本次运行结果。

## 对应源码

~~~matlab
function [ehat,m]=kf_error(y,previous,c,m)
% KF_ERROR Optional time-varying error-state Kalman filter (not a pose EKF).
if strcmp(c.filter.mode,'none'), ehat=y; m=[]; return; end
assert(~isempty(c.filter.Qn) && ~isempty(c.filter.Rn), ...
    'P7:KalmanCovarianceUnset', ...
    'Fill c.filter.Qn (scalar) and c.filter.Rn (3-by-3) before enabling KF.');
%% 1. Prediction from the previous APPLIED command, not unknown actuator noise
if isempty(m)
    m.mean=c.filter.initialMean; m.P=c.filter.P0;
else
    [Ad,Bd,dd]=p7utils.linear_step(previous.wr,c.v0,previous.vr,c.Ts);
    m.mean=Ad*m.mean+Bd*previous.u+dd;
    Qx=Bd*(c.filter.qScale*c.filter.Qn)*Bd';
    m.P=Ad*m.P*Ad'+Qx;
end
%% 2. Full error measurement update with wrapped heading innovation
Rn=c.filter.rScale*c.filter.Rn;
innovation=y-m.mean;
innovation(3)=atan2(sin(innovation(3)),cos(innovation(3)));
L=m.P/(m.P+Rn);
m.mean=m.mean+L*innovation;
m.mean(3)=atan2(sin(m.mean(3)),cos(m.mean(3)));
J=eye(3)-L;
m.P=J*m.P*J'+L*Rn*L';
m.P=(m.P+m.P')/2;
m.L=L;
ehat=m.mean;
end
~~~
