# predict_error_Project7.m 逐段说明

**作用：**MPC 单步误差预测：默认 LTV，或可选完整非线性误差动力学。

**调用关系：**objfun_Project7_MPC 调用。

**边界：**不在优化过程中 wrap 预测航向误差，避免人为分支跳跃；大角度情况仍需单独评价。

代码中的 %% 是主要逻辑分段；下面按执行顺序解释。短辅助函数没有强行拆成多段。完整源码附在末尾，便于逐项核对。

### 第 1 段：实现与原理

ltv 分支使用预先计算的 Ad、Bd、仿射项。

### 第 2 段：实现与原理

nonlinear 分支保留三角函数和输入乘误差项，并用 RK4 小步积分。

### 第 3 段：实现与原理

预测状态始终为误差；实际机器人由 compute_next_pose_Project7 或 Q5 pose_step 推进。

## 验证状态

MATLAB 语法树解析通过；相关数学使用 Python/SciPy 作了独立核对。没有在交付环境执行 MATLAB 或 Simulink，具体运行检查见包内 VALIDATION.md。对原平台复制函数，本次只做源码、调用和语法核对，未将原作者的静态记录当作本次运行结果。

## 对应源码

~~~matlab
function next=predict_error_Project7(e,u,p,j)
% PREDICT_ERROR_PROJECT7 Predict the ERROR state, not global pose.
%% 1. Frozen-interval LTV prediction
if strcmp(p.mode,'ltv')
    next=p.Ad(:,:,j)*e+p.Bd(:,j)*u+p.dd(:,j);
    return
end
%% 2. Optional nonlinear error model, integrated with RK4
w=p.wr(j)+u; vr=p.vr(j); v0=p.v0;
f=@(x)[w*x(2)-v0+vr*cos(x(3));-w*x(1)+vr*sin(x(3));-u];
h=p.Ts/p.predictionSubsteps; next=e;
for k=1:p.predictionSubsteps
    a=f(next); b=f(next+h*a/2); cc=f(next+h*b/2); d=f(next+h*cc);
    next=next+h*(a+2*b+2*cc+d)/6;
end
end
~~~
