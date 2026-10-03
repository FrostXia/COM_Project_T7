# export_noise.m 逐段说明

**作用：**保存可复核的完整 MAT、每次运行的 CSV 和汇总表。

**调用关系：**由 run_cases 调用。

**边界：**输出是运行时产生的结果；交付包不会预置虚构的 MATLAB 仿真图。

代码中的 %% 是主要逻辑分段；下面按执行顺序解释。短辅助函数没有强行拆成多段。完整源码附在末尾，便于逐项核对。

### 第 1 段：实现与原理

新建独立结果目录，避免覆盖先前实验。

### 第 2 段：实现与原理

状态日志有 M+1 点、控制有 M 个区间；末行控制明确填 NaN，不伪造一次新控制。

### 第 3 段：实现与原理

完整 MAT 包含配置、设计、噪声序列、参考残差、失败原因；CSV 用于后续统计。

## 验证状态

MATLAB 语法树解析通过；相关数学使用 Python/SciPy 作了独立核对。没有在交付环境执行 MATLAB 或 Simulink，具体运行检查见包内 VALIDATION.md。对原平台复制函数，本次只做源码、调用和语法核对，未将原作者的静态记录当作本次运行结果。

## 对应源码

~~~matlab
function folder=export_noise(runs,c,label)
% EXPORT_NOISE Save reproducible MAT, time series and an explicit summary.
if ~isfolder(c.outputRoot), mkdir(c.outputRoot); end
folder=tempname(c.outputRoot); mkdir(folder);
save(fullfile(folder,'analysis.mat'),'runs','c','label');
rows=cell(numel(runs),14);
for j=1:numel(runs)
    r=runs{j}; m=r.metrics; L=r.log; n=numel(L.t);
    name=[r.controller '_' r.scenario];
    pad=@(a)[a(:);nan(n-numel(a),1)];
    T=table(L.t(:),L.qTrue(1,:)',L.qTrue(2,:)',L.qTrue(3,:)', ...
        L.errorTrue(1,:)',L.errorTrue(2,:)',L.errorTrue(3,:)', ...
        L.errorMeasured(1,:)',L.errorMeasured(2,:)',L.errorMeasured(3,:)', ...
        L.errorEstimated(1,:)',L.errorEstimated(2,:)',L.errorEstimated(3,:)', ...
        pad(L.omegaRequested),pad(L.omegaCommand),pad(L.omegaPhysical), ...
        pad(L.omegaReference),pad(L.controlNoise),pad(L.solveSeconds),pad(L.exitflag), ...
        'VariableNames',{'time','x','y','theta','xeTrue','yeTrue','thetaeTrue', ...
        'xeMeasured','yeMeasured','thetaeMeasured','xeEstimated','yeEstimated','thetaeEstimated', ...
        'omegaRequested','omegaCommand','omegaPhysical','omegaReference','controlNoise','solveSeconds','exitflag'});
    writetable(T,fullfile(folder,[name '.csv']));
    rows(j,:)={r.controller,r.scenario,r.status,r.message,r.completed,r.planned, ...
        m.positionRMSE,m.headingRMSE,m.variance(1),m.variance(2),m.variance(3), ...
        m.saturationFraction,m.meanSolveSeconds,m.maxSolveSeconds};
end
S=cell2table(rows,'VariableNames',{'controller','scenario','status','message','completed','planned', ...
    'positionRMSE','headingRMSE','varXe','varYe','varThetae','saturationFraction','meanSolveSeconds','maxSolveSeconds'});
writetable(S,fullfile(folder,'summary.csv'));
disp(S); fprintf('Saved results: %s\n',folder);
end
~~~
