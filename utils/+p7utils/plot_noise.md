# plot_noise.m 逐段说明

**作用：**绘制轨迹、真值误差、测量/估计误差、控制输入和三轴方差。

**调用关系：**由 run_cases 调用；图中控制器/噪声标签对应原始结果。

**边界：**大量组合时图例可能较长，可通过 c.scenarios 减少显示工况。

代码中的 %% 是主要逻辑分段；下面按执行顺序解释。短辅助函数没有强行拆成多段。完整源码附在末尾，便于逐项核对。

### 第 1 段：实现与原理

参考与实际轨迹同图；误差绘制 true error，防止将滤波后的平滑当成跟踪精度。

### 第 2 段：实现与原理

最后一个运行展示指令和扰动后的物理角速度，以及横向真值/测量/估计误差。

### 第 3 段：实现与原理

三种误差方差使用分开的坐标轴和单位。

## 验证状态

MATLAB 语法树解析通过；相关数学使用 Python/SciPy 作了独立核对。没有在交付环境执行 MATLAB 或 Simulink，具体运行检查见包内 VALIDATION.md。对原平台复制函数，本次只做源码、调用和语法核对，未将原作者的静态记录当作本次运行结果。

## 对应源码

~~~matlab
function figs=plot_noise(runs,label)
% PLOT_NOISE Plot actual/reference paths, true errors, commands and variance.
if nargin<2, label='Project 7'; end
names=cellfun(@(x)[x.controller '/' x.scenario],runs,'UniformOutput',false);
figs=gobjects(3,1);
%% 1. Trajectories and true error histories
figs(1)=figure('Name',[label ' trajectories and errors']);
for a=1:4
    subplot(2,2,a); hold on; grid on;
    if a==1
        R=runs{1}.log.reference;
        plot(R(1,:),R(2,:),'k--','DisplayName','reference');
        for j=1:numel(runs), L=runs{j}.log; plot(L.qTrue(1,:),L.qTrue(2,:),'DisplayName',names{j}); end
        axis equal; xlabel('x (m)'); ylabel('y (m)'); title('Actual trajectories');
    else
        for j=1:numel(runs), L=runs{j}.log; plot(L.t,L.errorTrue(a-1,:),'DisplayName',names{j}); end
        xlabel('Time (s)'); labels={'x_e (m)','y_e (m)','theta_e (rad)'};
        ylabel(labels{a-1}); title('True tracking error');
    end
end
legend('Location','best');
%% 2. Physical input and measured/estimated errors for the final run
figs(2)=figure('Name',[label ' noise channels']);
L=runs{end}.log;
subplot(2,1,1); hold on; grid on;
if ~isempty(L.omegaCommand)
    tt=L.t(1:numel(L.omegaCommand));
    plot(tt,L.omegaReference,'k--'); plot(tt,L.omegaCommand); plot(tt,L.omegaPhysical);
end
legend('reference','command','physical'); ylabel('omega (rad/s)'); xlabel('Time (s)');
subplot(2,1,2); hold on; grid on;
plot(L.t,L.errorTrue(2,:)); plot(L.t,L.errorMeasured(2,:)); plot(L.t,L.errorEstimated(2,:));
legend('true y_e','measured y_e','used/estimated y_e'); xlabel('Time (s)'); ylabel('m');
%% 3. Variance has separate units for position and heading
figs(3)=figure('Name',[label ' variance']);
vv=cell2mat(cellfun(@(x)x.metrics.variance,runs,'UniformOutput',false)).';
for a=1:3
    subplot(3,1,a); bar(vv(:,a)); grid on;
    set(gca,'XTick',1:numel(names),'XTickLabel',names,'XTickLabelRotation',30);
    units={'var(x_e), m^2','var(y_e), m^2','var(theta_e), rad^2'};
    ylabel(units{a});
end
end
~~~
