# project7_mpc_step.m 逐段说明

**作用：**将 MPC 接入 TrajectoryLab 的 qHat/ref/config/memory 接口。

**调用关系：**由 bridge/p7_external_controller.m 调用。

**边界：**GUI 自己的估计器/噪声模型保持原样；任务3采用另外的 Q5 误差噪声实验。path 模式中的进度选择仍由平台负责，并未优化进度。

代码中的 %% 是主要逻辑分段；下面按执行顺序解释。短辅助函数没有强行拆成多段。完整源码附在末尾，便于逐项核对。

### 第 1 段：实现与原理

只读取公开配置、估计位姿和参考，不读取 qTrue 或仿真内部会话。

### 第 2 段：实现与原理

将 qHat 与当前参考转换为本体误差，随后调用公共误差 MPC。

### 第 3 段：实现与原理

输出物理 omega；将预测误差反变换为全局预测轨迹，便于平台记录。

## 验证状态

MATLAB 语法树解析通过；相关数学使用 Python/SciPy 作了独立核对。没有在交付环境执行 MATLAB 或 Simulink，具体运行检查见包内 VALIDATION.md。对原平台复制函数，本次只做源码、调用和语法核对，未将原作者的静态记录当作本次运行结果。

## 对应源码

~~~matlab
function [omega,memory,info]=project7_mpc_step(qHat,ref,config,memory)
% PROJECT7_MPC_STEP TrajectoryLab's global-pose controller interface.
%% 1. Map public platform configuration into the shared MPC configuration
c=p7utils.defaults;
c.v0=config.v0; c.vr=config.v0;
c.Ts=config.Ts; c.N=config.N;
c.wMin=config.wMin; c.wMax=config.wMax;
if isfield(config,'algorithm')
    names=fieldnames(config.algorithm);
    for j=1:numel(names)
        assert(isfield(c.mpc,names{j}),'P7:Algorithm','Unknown MPC option.');
        c.mpc.(names{j})=config.algorithm.(names{j});
    end
end
validateattributes(qHat,{'double'},{'size',[3 1],'finite','real'});
assert(isequal(size(ref.q),[3 c.N+1]),'P7:Preview','ref.q must be 3-by-(N+1).');
%% 2. Convert estimated global pose to body-frame tracking error
e0=p7utils.error_state(qHat,ref.q(:,1));
[omega,memory,info]=p7_mpc_error_step(e0,ref.w,c.vr*ones(c.N,1),c,memory);
%% 3. Optional predicted global pose for the platform's diagnostics
info.predictedQ=[];
if info.ok
    E=info.predictedError; qr=ref.q;
    th=qr(3,:)-E(3,:);
    xx=qr(1,:)-cos(th).*E(1,:)+sin(th).*E(2,:);
    yy=qr(2,:)-sin(th).*E(1,:)-cos(th).*E(2,:);
    info.predictedQ=[xx;yy;th];
end
end
~~~
