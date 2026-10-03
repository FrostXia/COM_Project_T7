# run_cases.m 逐段说明

**作用：**为各控制器和噪声工况使用同一条参考，启动相互独立的 Q5 循环。

**调用关系：**被两个任务3入口及任务2独立示例调用。

**边界：**c.plot/c.save 可以关闭，以便短测试和参数扫描。

代码中的 %% 是主要逻辑分段；下面按执行顺序解释。短辅助函数没有强行拆成多段。完整源码附在末尾，便于逐项核对。

### 第 1 段：实现与原理

一次生成参考；只有 SF/LQR 才调用任务1计算固定增益。

### 第 2 段：实现与原理

逐个运行 clean、observer、control、both 等工况；每次有独立控制器/估计器 memory。

### 第 3 段：实现与原理

显示状态与 RMSE，按配置绘图和导出。

## 验证状态

MATLAB 语法树解析通过；相关数学使用 Python/SciPy 作了独立核对。没有在交付环境执行 MATLAB 或 Simulink，具体运行检查见包内 VALIDATION.md。对原平台复制函数，本次只做源码、调用和语法核对，未将原作者的静态记录当作本次运行结果。

## 对应源码

~~~matlab
function runs=run_cases(c,inputPath,controllers,label)
% RUN_CASES One reference, one configuration, independent memories per run.
if nargin<2, inputPath=[]; end
p7utils.validate(c);
reference=p7utils.reference_data(c,inputPath);
design=[];
if any(ismember(controllers,{'sf','lqr'}))
    [~,~,design]=project7_q4_gains(c.design,false);
end
if any(strcmp(controllers,'mpc'))
    assert(exist('fmincon','file')~=0,'P7:Toolbox','MPC requires Optimization Toolbox (fmincon).');
end
runs=cell(1,numel(controllers)*numel(c.scenarios)); n=0;
for j=1:numel(controllers)
    for k=1:numel(c.scenarios)
        n=n+1;
        runs{n}=p7utils.simulate_q5(c,reference,controllers{j},c.scenarios{k},design);
        fprintf('%s / %s: %s, position RMSE %.6g m\n',controllers{j}, ...
            c.scenarios{k},runs{n}.status,runs{n}.metrics.positionRMSE);
    end
end
if c.plot, p7utils.plot_noise(runs,label); end
if c.save, p7utils.export_noise(runs,c,label); end
end
~~~
