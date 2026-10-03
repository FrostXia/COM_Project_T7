# objfun_Project7_MPC.m 逐段说明

**作用：**计算预测误差和角速度修正量的二次代价，并提供 LTV 的解析梯度。

**调用关系：**调用 predict_error_Project7；被 fmincon 多次评价，但不读写文件。

**边界：**Q/R 同数值不意味着与无限时域 LQR 等价。

代码中的 %% 是主要逻辑分段；下面按执行顺序解释。短辅助函数没有强行拆成多段。完整源码附在末尾，便于逐项核对。

### 第 1 段：实现与原理

只优化 N 个标量 u；从当前误差递推 N 步状态。

### 第 2 段：实现与原理

每步成本是 Ts*(e_next^T Q e_next + R*u²)，与原案例下一步状态计价方式对应。

### 第 3 段：实现与原理

LTV 分支传播状态对整个 U 的灵敏度，得到解析梯度；非线性模式使用求解器差分。

### 第 4 段：实现与原理

终端权重单独设置，默认零；有此选项不等于已证明稳定。

## 验证状态

MATLAB 语法树解析通过；相关数学使用 Python/SciPy 作了独立核对。没有在交付环境执行 MATLAB 或 Simulink，具体运行检查见包内 VALIDATION.md。对原平台复制函数，本次只做源码、调用和语法核对，未将原作者的静态记录当作本次运行结果。

## 对应源码

~~~matlab
function [f,g,E]=objfun_Project7_MPC(U,e0,p)
% OBJFUN_PROJECT7_MPC Roll out states internally; optimize controls only.
U=U(:); E=zeros(3,p.N+1); E(:,1)=e0;
f=0; g=zeros(p.N,1); sensitivity=zeros(3,p.N);
%% 1. Tracking and correction-input cost over the prediction horizon
for j=1:p.N
    E(:,j+1)=predict_error_Project7(E(:,j),U(j),p,j);
    f=f+p.Ts*(E(:,j+1)'*p.Q*E(:,j+1)+p.R*U(j)^2);
    if strcmp(p.mode,'ltv')
        sensitivity=p.Ad(:,:,j)*sensitivity;
        sensitivity(:,j)=sensitivity(:,j)+p.Bd(:,j);
        g=g+2*p.Ts*sensitivity'*p.Q*E(:,j+1);
        g(j)=g(j)+2*p.Ts*p.R*U(j);
    end
end
%% 2. Optional terminal cost; default is zero, not a stability certificate
f=f+E(:,end)'*p.terminalWeight*E(:,end);
if strcmp(p.mode,'ltv')
    g=g+2*sensitivity'*p.terminalWeight*E(:,end);
else
    g=[]; % fmincon finite differences are selected for the nonlinear model.
end
end
~~~
