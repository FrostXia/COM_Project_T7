# p7_mpc_error_step.m 逐段说明

**作用：**公共 MPC 求解器：从误差与未来参考角速度求第一步物理角速度。

**调用关系：**GUI 的 project7_mpc_step 和任务3 simulate_q5 均调用此同一函数。

**边界：**这是普通名义 MPC，不是随机/鲁棒 MPC；不能保证扰动后的物理角速度始终满足指令约束。

代码中的 %% 是主要逻辑分段；下面按执行顺序解释。短辅助函数没有强行拆成多段。完整源码附在末尾，便于逐项核对。

### 第 1 段：实现与原理

验证输入维度，并在求解器循环外构造 LTV 离散矩阵。

### 第 2 段：实现与原理

把 omega_min/max 转换成每步 u 的边界；前一次物理角速度计划移位后减去新的 wr，构造 warm start。

### 第 3 段：实现与原理

使用 fmincon 的 SQP；LTV 提供解析梯度。显式检查 exitflag、有限数值和边界残差。

### 第 4 段：实现与原理

失败返回 info.ok=false 和 NaN，不暗中使用演示控制器。

### 第 5 段：实现与原理

只执行第一个控制量；保存整段物理角速度计划用于下一步初始化。

## 验证状态

MATLAB 语法树解析通过；相关数学使用 Python/SciPy 作了独立核对。没有在交付环境执行 MATLAB 或 Simulink，具体运行检查见包内 VALIDATION.md。对原平台复制函数，本次只做源码、调用和语法核对，未将原作者的静态记录当作本次运行结果。

## 对应源码

~~~matlab
function [omega,memory,info]=p7_mpc_error_step(e0,wr,vr,c,memory)
% P7_MPC_ERROR_STEP Shared MPC solver for the GUI and the Q5 noise runner.
if nargin<5, memory=[]; end
omega=NaN;
info=struct('ok',false,'message','','exitflag',NaN,'constraintResidual',NaN, ...
    'predictedError',[],'objective',NaN,'solveSeconds',NaN);
%% 1. Explicit dimensions and model configuration
assert(exist('fmincon','file')~=0,'P7:Toolbox','MPC requires fmincon.');
validateattributes(e0,{'double'},{'size',[3 1],'finite','real'});
wr=wr(:); vr=vr(:);
assert(numel(wr)==c.N && numel(vr)==c.N && all(isfinite([wr;vr])), ...
    'P7:Preview','Reference must contain N finite angular rates and speeds.');
p=c.mpc; p.N=c.N; p.Ts=c.Ts; p.v0=c.v0; p.wr=wr; p.vr=vr;
assert(ismember(p.mode,{'ltv','nonlinear'}),'P7:MPC','Unknown prediction mode.');
assert(isequal(size(p.Q),[3 3]) && all(isfinite(p.Q(:))) && ...
    norm(p.Q-p.Q','fro')<1e-10 && min(eig(p.Q))>=0,'P7:Q','Q must be positive semidefinite.');
validateattributes(p.R,{'double'},{'scalar','real','finite','positive'});
assert(isequal(size(p.terminalWeight),[3 3]) && all(isfinite(p.terminalWeight(:))) && ...
    norm(p.terminalWeight-p.terminalWeight','fro')<1e-10 && ...
    min(eig(p.terminalWeight))>=0,'P7:Terminal','Invalid terminal weight.');
validateattributes(p.predictionSubsteps,{'double'},{'scalar','integer','positive'});
p.Ad=zeros(3,3,p.N); p.Bd=zeros(3,p.N); p.dd=zeros(3,p.N);
if strcmp(p.mode,'ltv')
    for j=1:p.N
        [p.Ad(:,:,j),p.Bd(:,j),p.dd(:,j)]=p7utils.linear_step(wr(j),p.v0,vr(j),p.Ts);
    end
end
%% 2. Physical omega bounds transformed into correction-input bounds
lb=c.wMin-wr; ub=c.wMax-wr;
U0=zeros(p.N,1);
if isstruct(memory) && isfield(memory,'omegaPlan') && numel(memory.omegaPlan)==p.N
    old=memory.omegaPlan(:);
    U0=[old(2:end);old(end)]-wr;
end
U0=min(max(U0,lb),ub);
%% 3. Receding-horizon optimization; no fallback controller is hidden
opts=optimoptions('fmincon','Algorithm','sqp','Display','none', ...
    'MaxIterations',p.maxIterations,'ConstraintTolerance',p.tolerance, ...
    'OptimalityTolerance',p.tolerance, ...
    'SpecifyObjectiveGradient',strcmp(p.mode,'ltv'));
clock=tic;
try
    [U,fval,flag,output]=fmincon(@(z)objfun_Project7_MPC(z,e0,p),U0, ...
        [],[],[],[],lb,ub,@confun_Project7_MPC,opts);
catch ME
    info.message=ME.message; info.solveSeconds=toc(clock); return
end
info.solveSeconds=toc(clock); info.exitflag=flag; info.objective=fval;
if numel(U)~=p.N || any(~isfinite(U(:))) || ~isfinite(fval)
    info.message='Solver returned an invalid candidate.'; return
end
U=U(:);
res=max([0;lb-U;U-ub]);
[~,~,E]=objfun_Project7_MPC(U,e0,p);
info.constraintResidual=res; info.predictedError=E;
info.ok=flag>0 && res<=10*p.tolerance && all(isfinite(E(:)));
if ~info.ok
    info.message=sprintf('MPC rejected: exitflag=%d, bound residual=%.3g. %s',flag,res,output.message);
    return
end
%% 4. Execute only the first physical angular rate and retain a warm start
omega=wr(1)+U(1);
memory=struct('omegaPlan',wr+U,'inputPlan',U,'lastObjective',fval);
info.message=sprintf('%s MPC accepted, exitflag=%d.',p.mode,flag);
end
~~~
