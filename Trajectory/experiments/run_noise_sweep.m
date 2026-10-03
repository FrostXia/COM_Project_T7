function study=run_noise_sweep(kind,valuesA,valuesB,c,inputPath)
% RUN_NOISE_SWEEP Vary one SF pole, or KF Qn/Rn scales with true noise fixed.
addpath(fileparts(fileparts(fileparts(mfilename('fullpath')))));
setup_project7;
if nargin<1 || isempty(kind), kind='lqr_kf'; end
if nargin<2 || isempty(valuesA)
    if strcmp(kind,'pole'), valuesA=[-0.5 -1 -1.5]; else, valuesA=[0.1 1 10]; end
end
if nargin<3 || isempty(valuesB), valuesB=[0.1 1 10]; end
if nargin<4 || isempty(c), c=p7utils.defaults; end
if nargin<5, inputPath=[]; end
assert(ismember(kind,{'pole','lqr_kf','mpc_kf'}),'P7:Sweep','Use pole, lqr_kf or mpc_kf.');
% Reject an unconfigured KF scan before building a reference or running cases.
if ~strcmp(kind,'pole')
    check=c; check.filter.mode='kf'; p7utils.validate(check);
end
%% 1. Preserve one reference and the actual noise; only selected design settings vary
reference=p7utils.reference_data(c,inputPath);
if strcmp(kind,'pole'), valuesB=1; end
if ~strcmp(kind,'pole')
    assert(all(valuesA>0) && all(valuesB>0),'P7:Sweep','Covariance scales must be positive.');
end
runs=cell(numel(valuesB),numel(valuesA));
score=nan(size(runs)); variances=nan(numel(valuesB),numel(valuesA),3);
for b=1:numel(valuesB)
    for a=1:numel(valuesA)
        cc=c; cc.plot=false; cc.save=false; design=[];
        if strcmp(kind,'pole')
            index=1;
            if isfield(c,'sweepPoleIndex'), index=c.sweepPoleIndex; end
            validateattributes(index,{'double'},{'scalar','integer','>=',1,'<=',3});
            cc.design.poles(index)=valuesA(a);
            assert(numel(unique(cc.design.poles))==3,'P7:Sweep','Use distinct single-input place poles.');
            [~,~,design]=project7_q4_gains(cc.design,false);
            controller='sf';
        else
            cc.filter.mode='kf';
            cc.filter.qScale=valuesA(a); cc.filter.rScale=valuesB(b);
            if strcmp(kind,'lqr_kf')
                controller='lqr'; [~,~,design]=project7_q4_gains(cc.design,false);
            else
                controller='mpc';
                assert(exist('p7_mpc_error_step','file')~=0,'P7:Package','Use the MPC noise package for mpc_kf.');
            end
        end
        r=p7utils.simulate_q5(cc,reference,controller,'both',design);
        runs{b,a}=r;
        if r.metrics.eligibleForComparison
            score(b,a)=r.metrics.positionRMSE;
            variances(b,a,:)=reshape(r.metrics.variance,1,1,3);
        end
    end
end
%% 2. Best means lowest position RMSE within this finite completed grid only
study=struct('kind',kind,'valuesA',valuesA,'valuesB',valuesB,'runs',{runs}, ...
    'positionRMSE',score,'variance',variances,'best',[]);
valid=find(isfinite(score));
if ~isempty(valid)
    [~,k]=min(score(valid)); [b,a]=ind2sub(size(score),valid(k));
    study.best=struct('valueA',valuesA(a),'valueB',valuesB(b),'positionRMSE',score(b,a));
end
%% 3. Pole curve or separate 3D covariance surfaces, without mixing units
if c.plot
    figure('Name',['Project 7 ' kind]);
    if strcmp(kind,'pole')
        subplot(2,1,1); plot(valuesA,score(1,:),'-o'); grid on;
        xlabel('Varied pole (1/s)'); ylabel('Position RMSE (m)');
        subplot(2,1,2); plot(valuesA,squeeze(variances(1,:,1:2)),'-o'); grid on;
        xlabel('Varied pole (1/s)'); ylabel('Position error variance (m^2)'); legend('x_e','y_e');
    else
        [AA,BB]=meshgrid(valuesA,valuesB);
        z={score,variances(:,:,1),variances(:,:,2),variances(:,:,3)};
        titles={'Position RMSE (m)','var(x_e) (m^2)','var(y_e) (m^2)','var(theta_e) (rad^2)'};
        for j=1:4
            subplot(2,2,j); surf(AA,BB,z{j}); grid on;
            xlabel('Qn scale'); ylabel('Rn scale'); zlabel(titles{j}); title(titles{j});
        end
    end
end
if c.save
    if ~isfolder(c.outputRoot), mkdir(c.outputRoot); end
    folder=tempname(c.outputRoot); mkdir(folder);
    save(fullfile(folder,'sweep.mat'),'study','c');
    [AA,BB]=meshgrid(valuesA,valuesB);
    T=table(AA(:),BB(:),score(:),'VariableNames',{'parameterA','parameterB','positionRMSE'});
    writetable(T,fullfile(folder,'sweep.csv'));
    fprintf('Sweep saved: %s\n',folder);
end
disp(study.best);
end
