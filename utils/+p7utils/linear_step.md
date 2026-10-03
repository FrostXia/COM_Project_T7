# linear_step.m 逐段说明

**作用：**离散化线性误差模型，并显式保留 vr-v0 的仿射失配。

**调用关系：**供 LTV MPC 和可选 KF 使用。

**边界：**对每个采样区间冻结 wr；速度失配时原点不再是平衡点。

代码中的 %% 是主要逻辑分段；下面按执行顺序解释。短辅助函数没有强行拆成多段。完整源码附在末尾，便于逐项核对。

### 第 1 段：实现与原理

A 的航向耦合项使用实际参考速度 vr；基准 vr=v0 时回到既定模型。

### 第 2 段：实现与原理

增广矩阵指数同时计算 Ad、Bd 和仿射项，避免对奇异 A 求逆。

## 验证状态

MATLAB 语法树解析通过；相关数学使用 Python/SciPy 作了独立核对。没有在交付环境执行 MATLAB 或 Simulink，具体运行检查见包内 VALIDATION.md。对原平台复制函数，本次只做源码、调用和语法核对，未将原作者的静态记录当作本次运行结果。

## 对应源码

~~~matlab
function [Ad,Bd,dd]=linear_step(wr,v0,vr,Ts)
% LINEAR_STEP Freeze the reference over one interval, then use an exponential.
A=[0 wr 0;-wr 0 vr;0 0 0]; B=[0;0;-1]; d=[vr-v0;0;0];
E=expm([A B d;zeros(2,5)]*Ts);
Ad=E(1:3,1:3); Bd=E(1:3,4); dd=E(1:3,5);
end
~~~
