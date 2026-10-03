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
y(:,1)=e(:,1)+V(:,1); y(3,1)=p7_wrap(y(3,1));
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
            case {'sf','lqr'}
                feedbackConfig=struct('controller',controller);
                [omega,~,info]=project7_feedback_step(eh(:,k),wr,feedbackConfig,struct('design',design));
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
    y(3,k+1)=p7_wrap(y(3,k+1));
    previous=struct('u',command(k)-wr,'wr',wr,'vr',reference.vr(k));
    completed=k;
end
%% 4. Final measurement correction and unambiguous logs
if completed==M
    [eh(:,n),~]=p7utils.kf_error(y(:,n),previous,c,filterMemory);
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
