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
