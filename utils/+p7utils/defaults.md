# defaults.m 逐段说明

**作用：**统一三个工程的仿真默认值；设计权重从任务1读取。

**调用关系：**project7_q4_gains(...,parametersOnly=true)。

**边界：**Ts/N/限幅/噪声数值为可修改仿真初值；只有截图中的设计值已由本次任务指定。

代码中的 %% 是主要逻辑分段；下面按执行顺序解释。短辅助函数没有强行拆成多段。完整源码附在末尾，便于逐项核对。

### 第 1 段：实现与原理

设计参数与实际 v0、参考 vr 分开：速度失配实验不暗中重新设计 K。

### 第 2 段：实现与原理

测量噪声三维、控制扰动一维；标准差沿用 Q5 量级，初始采样默认不加测量噪声。

### 第 3 段：实现与原理

默认不滤波，保留 Q5 直接带噪误差反馈；c.filter.Qn=[]、Rn=[] 均待填写。实际注入噪声标准差与滤波协方差相互独立，不从前者自动填入后者。

### 第 4 段：实现与原理

默认 LTV MPC、N=20、Ts=0.1、零额外终端代价；nonlinear 是可选对照模式。

### 第 5 段：实现与原理

结果窗口、绘图、输出路径可以手动调整，默认评价完整运行。

## 验证状态

MATLAB 语法树解析通过；相关数学使用 Python/SciPy 作了独立核对。没有在交付环境执行 MATLAB 或 Simulink，具体运行检查见包内 VALIDATION.md。对原平台复制函数，本次只做源码、调用和语法核对，未将原作者的静态记录当作本次运行结果。

## 对应源码

~~~matlab
function c=defaults
% DEFAULTS Common experiment settings; algorithm parameters come from Task 1.
%% 1. Fixed design parameters and physical/reference speeds
[~,~,d]=project7_q4_gains([],false,true);
c.design=d.parameters;
c.v0=d.parameters.v0; c.vr=c.v0;
c.Ts=0.1; c.N=20; c.duration=30;
c.q0=[0;0;0]; c.qr0=[0;0;0];
c.referenceKind='q4circle';
c.pathParams=[12 3 0.15 1];
c.integrationSteps=100;
%% 2. Noise conventions: independent measurement and angular-rate disturbance
c.measurementStd=[0.05;0.05;0.05];
c.controlStd=0.1;
c.noiseAtInitialSample=false; % Match the uncommented Q5 initial state
c.seed=42;
c.scenarios={'clean','observer','control','both'};
c.wMin=-1.2; c.wMax=1.2;
c.applyCommandLimits=true;
% Noise is added AFTER command limiting; the disturbed physical rate is not clipped.
%% 3. Optional error-state Kalman filter; raw measurements are the Q5 default
c.filter.mode='none'; % 'none' or 'kf'
c.filter.Qn=[]; % USER TO FILL: scalar angular-rate noise variance
c.filter.Rn=[]; % USER TO FILL: 3-by-3 error-measurement covariance
c.filter.qScale=1; c.filter.rScale=1;
c.filter.initialMean=zeros(3,1);
c.filter.P0=diag([0.1 0.1 0.1].^2);
%% 4. MPC setup: LTV by default; optional nonlinear error prediction
c.mpc.mode='ltv'; % 'ltv' or 'nonlinear'
c.mpc.Q=d.parameters.Q; c.mpc.R=d.parameters.R;
c.mpc.terminalWeight=zeros(3);
c.mpc.maxIterations=100;
c.mpc.tolerance=1e-6;
c.mpc.predictionSubsteps=4;
%% 5. Path preparation, evaluation and output
c.pathSpacing=0.03; c.smoothingWindow=11;
c.maxPathDeviation=0.20; c.lookahead=0.8;
c.analysisStart=0; % All samples; change explicitly for a tail-window study
c.plot=true; c.save=true;
c.outputRoot=fullfile(pwd,'results');
end
~~~
