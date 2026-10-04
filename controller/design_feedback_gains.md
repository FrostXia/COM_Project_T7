# design_feedback_gains.m 逐段说明

**作用：**任务1：按 Q4 顺序计算当前单输入模型的 K、Klqr、闭环矩阵和极点。

**调用关系：**被 default_config、feedback_control_step 和界面设计检查调用。

**边界：**仅证明固定名义连续模型结果，不证明整条手绘路径稳定。

代码中的 %% 是主要逻辑分段；下面按执行顺序解释。短辅助函数没有强行拆成多段。完整源码附在末尾，便于逐项核对。

### 第 1 段：实现与原理

截图参数是唯一设计默认源。p 允许受控实验覆盖参数，不能传未识别字段。

### 第 2 段：实现与原理

构造 A、B，显式检查 [B,AB,A²B] 的秩；u=omega-omega_r 决定 B 的第三项为 -1。

### 第 3 段：实现与原理

place 根据指定 poles 求 K，并通过闭环特征多项式核对。

### 第 4 段：实现与原理

lqr 根据 Q、R 求 Klqr 和 Riccati 解，检查残差与闭环稳定性。

### 第 5 段：实现与原理

返回全精度数值，不把屏幕显示的四位小数作为仿真参数。parametersOnly=true 可让其他包读取默认参数而不调用工具箱。

## 验证状态

MATLAB 语法树解析通过；相关数学使用 Python/SciPy 作了独立核对。没有在交付环境执行 MATLAB 或 Simulink，具体运行检查见包内 VALIDATION.md。对原平台复制函数，本次只做源码、调用和语法核对，未将原作者的静态记录当作本次运行结果。

## 对应源码

~~~matlab
function [K,Klqr,design] = design_feedback_gains(p,verbose,parametersOnly)
% DESIGN_FEEDBACK_GAINS Q4-style continuous single-input Trajectory design.
% [K,Klqr,design] = design_feedback_gains
% Optional p overrides are for controlled parameter experiments.
%% 1. Screenshot parameters and explicit override validation
base=struct('v0',0.5,'wrNom',pi/4,'poles',[-1 -2 -4],'Q',eye(3),'R',1);
if nargin<1 || isempty(p), p=base; else
    names=fieldnames(p);
    assert(all(ismember(names,fieldnames(base))),'Trajectory:Parameter','Unknown design field.');
    for i=1:numel(names), base.(names{i})=p.(names{i}); end
    p=base;
end
if nargin<2, verbose=true; end
if nargin<3, parametersOnly=false; end
validateattributes(p.v0,{'double'},{'scalar','real','finite','positive'});
validateattributes(p.wrNom,{'double'},{'scalar','real','finite','nonzero'});
validateattributes(p.poles,{'double'},{'vector','numel',3,'finite'});
assert(all(real(p.poles)<0),'Trajectory:Poles','Continuous poles must have negative real parts.');
assert(isequal(size(p.Q),[3 3]) && isreal(p.Q) && all(isfinite(p.Q(:))) && ...
    norm(p.Q-p.Q','fro')<1e-10 && min(eig(p.Q))>0,'Trajectory:Q','This prototype requires Q positive definite.');
validateattributes(p.R,{'double'},{'scalar','real','finite','positive'});
%% 2. Nonlinear error convention u = omega - omegaReference
A=[0 p.wrNom 0;-p.wrNom 0 p.v0;0 0 0];
B=[0;0;-1];
Co=[B A*B A*A*B];
assert(rank(Co)==3,'Trajectory:Controllability','Nominal model is not fully controllable.');
design=struct('parameters',p,'A',A,'B',B,'controllability',Co);
K=[]; Klqr=[];
if parametersOnly, return; end
assert(exist('place','file')~=0 && exist('lqr','file')~=0, ...
    'Trajectory:Toolbox','Control System Toolbox is required for place and lqr.');
%% 3. Pole placement, as in Assignment 2 Q4
K=place(A,B,p.poles);
Acl=A-B*K;
design.K=K; design.Acl=Acl; design.polesSF=eig(Acl);
target=poly(p.poles); actual=poly(Acl);
assert(norm(actual-target)<1e-7*max(1,norm(target)), ...
    'Trajectory:Verification','Pole placement verification failed.');
%% 4. Continuous LQR and Riccati residual
[Klqr,Plqr,Elqr]=lqr(A,B,p.Q,p.R);
Acl_lqr=A-B*Klqr;
design.Klqr=Klqr; design.Plqr=Plqr;
design.Acl_lqr=Acl_lqr; design.polesLQR=Elqr;
design.careResidual=norm(A'*Plqr+Plqr*A-Plqr*B*(p.R\(B'*Plqr))+p.Q,'fro');
assert(all(real(eig(Acl_lqr))<0),'Trajectory:Verification','Nominal LQR is not stable.');
%% 5. Print matrices and poles; keep full precision in returned variables
if verbose
    fprintf('\nTrajectory: v0=%.6g m/s, nominal Wr=%.6g rad/s\n',p.v0,p.wrNom);
    disp('A ='); disp(A); disp('B ='); disp(B);
    disp('K ='); disp(K); disp('Acl = A-B*K ='); disp(Acl);
    disp('eig(Acl) ='); disp(design.polesSF);
    disp('Q ='); disp(p.Q); disp('R ='); disp(p.R);
    disp('Klqr ='); disp(Klqr); disp('Plqr ='); disp(Plqr);
    disp('Acl_lqr = A-B*Klqr ='); disp(Acl_lqr);
    disp('eig(Acl_lqr) ='); disp(Elqr);
    fprintf('CARE residual: %.3g\n',design.careResidual);
end
end
~~~
