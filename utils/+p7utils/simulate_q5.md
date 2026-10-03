# simulate_q5.m 逐段说明

**作用：**任务3共同仿真循环：完全沿用 Q5 的控制、噪声、运动、误差更新次序，改为固定速度单输入。

**调用关系：**调用 error_state、pose_step、kf_error、noise_metrics；MPC 调用同一个 p7_mpc_error_step。

**边界：**默认限幅是对 Q5 模板的显式工程扩展。c.applyCommandLimits=false 可作 SF/LQR 的原模板无约束对照；不能与有约束 MPC 直接称公平对比。

代码中的 %% 是主要逻辑分段；下面按执行顺序解释。短辅助函数没有强行拆成多段。完整源码附在末尾，便于逐项核对。

### 第 1 段：实现与原理

分别预生成测量与执行噪声；同一 seed 在不同控制器/扫描参数下得到相同原始随机序列。

### 第 2 段：实现与原理

将真值 q/e、测量 y 和可选估计 eh 分开保存；默认第一个误差测量无噪声，与原模板一致。

### 第 3 段：实现与原理

先由当前带噪/估计误差计算角速度，限幅指令后加入一个角速度扰动。不给线速度加噪声。

### 第 4 段：实现与原理

实际机器人用受扰角速度积分；参考始终无噪声；更新下一拍误差测量。

### 第 5 段：实现与原理

求解器或控制器失败停止并保留失败状态，不回退到 demo，不把失败点填成成功结果。

### 第 6 段：实现与原理

日志区分请求角速度、限幅指令、扰动后的物理角速度。KF 只能知道指令，不能读取真实扰动。

## 验证状态

MATLAB 语法树解析通过；相关数学使用 Python/SciPy 作了独立核对。没有在交付环境执行 MATLAB 或 Simulink，具体运行检查见包内 VALIDATION.md。对原平台复制函数，本次只做源码、调用和语法核对，未将原作者的静态记录当作本次运行结果。

## 对应源码

~~~matlab
function out=simulate_q5(c,reference,controller,scenario,design)
% SIMULATE_Q5 Q5 order, single fixed speed, separate true and measured errors.
if nargin<5, design=[]; end
p7utils.validate(c);
assert(ismember(controller,{'openloop','sf','lqr','mpc'}),'P7:Controller','Unknown controller.');
assert(ismember(scenario,{'clean','observer','control','both'}),'P7:Noise','Unknown scenario.');
if ismember(controller,{'sf','lqr'})
    assert(~isempty(design),'P7:Design','Pass the Task 1 design.');
end
if strcmp(controller,'mpc')
    assert(c.applyCommandLimits,'P7:Bounds','MPC comparison requires command limits enabled.');
end
%% 1. Independent, repeatable random streams generated before feedback
M=reference.steps; n=M+1;
sensorStream=RandStream('mt19937ar','Seed',c.seed);
controlStream=RandStream('mt19937ar','Seed',c.seed+1);
V=c.measurementStd.*randn(sensorStream,3,n);
W=c.controlStd*randn(controlStream,1,M);
if ~c.noiseAtInitialSample, V(:,1)=0; end
if ~ismember(scenario,{'observer','both'}), V(:)=0; end
if ~ismember(scenario,{'control','both'}), W(:)=0; end
%% 2. Initial true state and initial error measurement, as in Q5
q=nan(3,n); e=nan(3,n); y=nan(3,n); eh=nan(3,n);
requested=nan(1,M); command=nan(1,M); physical=nan(1,M);
solveSeconds=nan(1,M); exitflags=nan(1,M); boundResidual=nan(1,M);
ok=false(1,M);
q(:,1)=c.q0; e(:,1)=p7utils.error_state(q(:,1),reference.q(:,1));
y(:,1)=e(:,1)+V(:,1); y(3,1)=atan2(sin(y(3,1)),cos(y(3,1)));
filterMemory=[]; controlMemory=[]; previous=[];
completed=0; attempted=0; status='completed'; message='Configured horizon completed.';
%% 3. Q5 loop: control -> actuator noise -> plant -> reference -> noisy error
for k=1:M
    attempted=k;
    [eh(:,k),filterMemory]=p7utils.kf_error(y(:,k),previous,c,filterMemory);
    wr=reference.w(k); started=tic;
    info=struct('ok',true,'message','','exitflag',NaN,'constraintResidual',0);
    try
        switch controller
            case 'openloop'
                omega=wr;
            case 'sf'
                omega=wr-design.K*eh(:,k);
            case 'lqr'
                omega=wr-design.Klqr*eh(:,k);
            case 'mpc'
                [omega,controlMemory,info]=p7_mpc_error_step(eh(:,k), ...
                    reference.w(k:k+c.N-1),reference.vr(k:k+c.N-1),c,controlMemory);
        end
        if info.ok && (~isscalar(omega) || ~isreal(omega) || ~isfinite(omega))
            info.ok=false; info.message='Non-finite scalar control.';
        end
    catch ME
        omega=NaN; info.ok=false; info.message=ME.message;
    end
    solveSeconds(k)=toc(started);
    if isfield(info,'exitflag'), exitflags(k)=info.exitflag; end
    if isfield(info,'constraintResidual'), boundResidual(k)=info.constraintResidual; end
    requested(k)=omega; ok(k)=logical(info.ok);
    if ~info.ok
        status='failed'; message=info.message; break
    end
    if c.applyCommandLimits, omega=min(max(omega,c.wMin),c.wMax); end
    command(k)=omega;
    physical(k)=omega+W(k); % Add unknown disturbance AFTER the command
    q(:,k+1)=p7utils.pose_step(q(:,k),c.v0,physical(k),c.Ts,c.integrationSteps);
    % Reference is precomputed without any measurement or process noise.
    e(:,k+1)=p7utils.error_state(q(:,k+1),reference.q(:,k+1));
    y(:,k+1)=e(:,k+1)+V(:,k+1);
    y(3,k+1)=atan2(sin(y(3,k+1)),cos(y(3,k+1)));
    previous=struct('u',command(k)-wr,'wr',wr,'vr',reference.vr(k));
    completed=k;
end
%% 4. Final measurement correction and unambiguous logs
if completed==M
    [eh(:,n),filterMemory]=p7utils.kf_error(y(:,n),previous,c,filterMemory);
end
count=completed+1;
log=struct('t',reference.t(1:count),'qTrue',q(:,1:count), ...
    'reference',reference.q(:,1:count),'errorTrue',e(:,1:count), ...
    'errorMeasured',y(:,1:count),'errorEstimated',eh(:,1:count), ...
    'measurementNoise',V(:,1:count),'controlNoise',W(1:completed), ...
    'omegaRequested',requested(1:completed),'omegaCommand',command(1:completed), ...
    'omegaPhysical',physical(1:completed),'omegaReference',reference.w(1:completed), ...
    'solveSeconds',solveSeconds(1:completed),'exitflag',exitflags(1:completed), ...
    'boundResidual',boundResidual(1:completed));
out=struct('controller',controller,'scenario',scenario,'config',c, ...
    'design',design,'status',status,'message',message,'log',log, ...
    'attempted',attempted,'completed',completed,'planned',M, ...
    'lastRequested',requested(max(1,attempted)), ...
    'lastExitflag',exitflags(max(1,attempted)), ...
    'lastSolveSeconds',solveSeconds(max(1,attempted)), ...
    'referenceSource',reference.source,'referenceResidual',reference.maxModelResidual);
out.metrics=p7utils.noise_metrics(out);
end
~~~
