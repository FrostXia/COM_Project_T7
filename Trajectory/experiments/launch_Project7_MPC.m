function result=launch_Project7_MPC(c,inputPath)
% LAUNCH_PROJECT7_MPC Original-template workflow: load, preview, solve, move.
% launch_Project7_MPC(c,'trajectory.dat') runs a standalone clean experiment.
% For the GUI use connect_TrajectoryLab(platformRoot). The old directory-only
% launch_Project7_MPC(platformRoot) call is preserved as a compatibility route.
addpath(fileparts(fileparts(fileparts(mfilename('fullpath')))));
setup_project7;
if nargin>=1 && (ischar(c) || (isstring(c) && isscalar(c)))
    assert(nargin==1 && isfolder(c),'P7:Launch', ...
        'Use launch_Project7_MPC(config,pathFile) for a path file.');
    result=connect_TrajectoryLab(c); return
end
%% 1. Parameters and trajectory loading belong to the main workflow
if nargin<1 || isempty(c), c=p7utils.defaults; end
if nargin<2, inputPath=[]; end
c.filter.mode='none'; c.scenarios={'clean'};
assert(c.integrationSteps==100,'P7:Integration', ...
    'This template plant uses 100 Q5 substeps. Keep integrationSteps=100.');
assert(c.applyCommandLimits,'P7:Bounds','MPC requires command limits enabled.');
reference=p7utils.reference_data(c,inputPath);
M=reference.steps;
q=nan(3,M+1); e=nan(3,M+1); omega=nan(1,M);
q(:,1)=c.q0;
e(:,1)=p7utils.error_state(q(:,1),reference.q(:,1));
memory=[]; solver=cell(1,M);
completed=0; status='completed'; message='Configured horizon completed.';
%% 2. Receding-horizon loop: update error and select N future reference rates
for k=1:M
    index=k:k+c.N-1;
    e(:,k)=p7utils.error_state(q(:,k),reference.q(:,k));
    % The helper prepares bounds/options and invokes objfun/confun via fmincon.
    [w,memory,info]=p7_mpc_error_step(e(:,k), ...
        reference.w(index),reference.vr(index),c,memory);
    solver{k}=info;
    if ~info.ok
        status='failed'; message=info.message; break
    end
    %% 3. Apply only the first optimized physical input, then move the robot
    omega(k)=min(max(w,c.wMin),c.wMax);
    [theta,x,y]=compute_next_pose_Project7( ...
        q(3,k),q(1,k),q(2,k),c.v0,omega(k),c.Ts);
    q(:,k+1)=[x;y;theta];
    e(:,k+1)=p7utils.error_state(q(:,k+1),reference.q(:,k+1));
    completed=k;
end
%% 4. Preserve completion/failure status, plot, and export available samples
count=completed+1;
result=struct('config',c,'status',status,'message',message, ...
    'completed',completed,'planned',M,'t',reference.t(1:count), ...
    'q',q(:,1:count),'reference',reference.q(:,1:count), ...
    'error',e(:,1:count),'omega',omega(1:completed), ...
    'solver',{solver(1:min(completed+1,M))}, ...
    'referenceResidual',reference.maxModelResidual);
if c.plot
    figure('Name','Project 7 MPC');
    plot(result.q(1,:),result.q(2,:),'b', ...
        reference.q(1,1:M+1),reference.q(2,1:M+1),'r--');
    axis equal; grid on; xlabel('x (m)'); ylabel('y (m)');
    legend('Robot','Reference'); title(['Project 7 MPC: ' status]);
end
if c.save
    if ~isfolder(c.outputRoot), mkdir(c.outputRoot); end
    folder=tempname(c.outputRoot); mkdir(folder);
    save(fullfile(folder,'mpc_result.mat'),'result','reference');
    T=table(result.t.',result.q(1,:).',result.q(2,:).',result.q(3,:).', ...
        result.error(1,:).',result.error(2,:).',result.error(3,:).', ...
        'VariableNames',{'t','x','y','theta','xe','ye','thetae'});
    writetable(T,fullfile(folder,'trajectory.csv'));
    fprintf('MPC result saved: %s\n',folder);
end
fprintf('MPC %s: %d/%d steps. %s\n',status,completed,M,message);
end
