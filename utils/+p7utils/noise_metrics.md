# noise_metrics.m 逐段说明

**作用：**计算真值误差、方差、输入用量、饱和和求解时间指标。

**调用关系：**由 simulate_q5 调用，结果供表格、扫描图和导出使用。

**边界：**不同运行若终止时间不同，有限段指标不可直接用作最终优劣排序。

代码中的 %% 是主要逻辑分段；下面按执行顺序解释。短辅助函数没有强行拆成多段。完整源码附在末尾，便于逐项核对。

### 第 1 段：实现与原理

analysisStart 允许统一评价时间窗；默认包含全程，不把瞬态误差方差直接称为纯噪声方差。

### 第 2 段：实现与原理

分别给出均值、RMSE、方差，避免把有偏但平滑的曲线当作更好。

### 第 3 段：实现与原理

commandEnergy 是角速度平方积分的控制用量，不是真实电能。

### 第 4 段：实现与原理

失败运行仍有日志，但 eligibleForComparison=false，不能参加“最佳设置”评选。

## 验证状态

MATLAB 语法树解析通过；相关数学使用 Python/SciPy 作了独立核对。没有在交付环境执行 MATLAB 或 Simulink，具体运行检查见包内 VALIDATION.md。对原平台复制函数，本次只做源码、调用和语法核对，未将原作者的静态记录当作本次运行结果。

## 对应源码

~~~matlab
function m=noise_metrics(out)
% NOISE_METRICS True tracking quality, not just filtered-curve smoothness.
L=out.log; idx=L.t>=out.config.analysisStart;
m=struct('positionRMSE',NaN,'headingRMSE',NaN,'meanError',nan(3,1), ...
    'variance',nan(3,1),'measurementVariance',nan(3,1), ...
    'controlNoiseVariance',NaN,'commandEnergy',NaN,'correctionEnergy',NaN, ...
    'peakPhysicalOmega',NaN,'meanSolveSeconds',NaN,'maxSolveSeconds',NaN, ...
    'saturationFraction',NaN,'sampleCount',sum(idx), ...
    'eligibleForComparison',strcmp(out.status,'completed'));
if any(idx)
    E=L.errorTrue(:,idx);
    m.positionRMSE=sqrt(mean(sum(E(1:2,:).^2,1)));
    m.headingRMSE=sqrt(mean(E(3,:).^2));
    m.meanError=mean(E,2);
    if sum(idx)>1
        m.variance=var(E,0,2);
        m.measurementVariance=var(L.measurementNoise(:,idx),0,2);
    end
end
if ~isempty(L.omegaCommand)
    m.commandEnergy=sum(L.omegaCommand.^2)*out.config.Ts;
    m.correctionEnergy=sum((L.omegaCommand-L.omegaReference).^2)*out.config.Ts;
    m.peakPhysicalOmega=max(abs(L.omegaPhysical));
    m.meanSolveSeconds=mean(L.solveSeconds);
    m.maxSolveSeconds=max(L.solveSeconds);
    m.saturationFraction=mean(abs(L.omegaRequested-L.omegaCommand)>1e-9);
    if numel(L.controlNoise)>1, m.controlNoiseVariance=var(L.controlNoise,0,2); end
end
end
~~~
