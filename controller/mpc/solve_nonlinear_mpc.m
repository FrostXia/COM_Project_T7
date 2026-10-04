function [omega,memory,info]=solve_nonlinear_mpc(e0,wr,vr,c,memory)
% SOLVE_NONLINEAR_MPC Solve the nonlinear tracking-error MPC problem.
if nargin<5, memory=[]; end
omega=NaN;
info=struct('ok',false,'message','','exitflag',NaN,'constraintResidual',NaN, ...
    'predictedError',[],'objective',NaN,'solveSeconds',NaN);
%% 1. Explicit dimensions and model configuration
assert(exist('fmincon','file')~=0,'Trajectory:Toolbox','MPC requires fmincon.');
validateattributes(e0,{'double'},{'size',[3 1],'finite','real'});
wr=wr(:); vr=vr(:);
assert(numel(wr)==c.N && numel(vr)==c.N && all(isfinite([wr;vr])), ...
    'Trajectory:Preview','Reference must contain N finite angular rates and speeds.');
p=c.mpc; p.N=c.N; p.Ts=c.Ts; p.v0=c.v0; p.wr=wr; p.vr=vr;
assert(strcmp(p.mode,'nonlinear'),'Trajectory:MPC','Only nonlinear prediction is supported.');
assert(isequal(size(p.Q),[3 3]) && all(isfinite(p.Q(:))) && ...
    norm(p.Q-p.Q','fro')<1e-10 && min(eig(p.Q))>=0,'Trajectory:Q','Q must be positive semidefinite.');
validateattributes(p.R,{'double'},{'scalar','real','finite','positive'});
assert(isequal(size(p.terminalWeight),[3 3]) && all(isfinite(p.terminalWeight(:))) && ...
    norm(p.terminalWeight-p.terminalWeight','fro')<1e-10 && ...
    min(eig(p.terminalWeight))>=0,'Trajectory:Terminal','Invalid terminal weight.');
validateattributes(p.predictionSubsteps,{'double'},{'scalar','integer','positive'});
%% 2. Physical omega bounds transformed into correction-input bounds
lb=c.wMin-wr; ub=c.wMax-wr;
U0=zeros(p.N,1);
if isstruct(memory) && isfield(memory,'omegaPlan') && numel(memory.omegaPlan)==p.N
    old=memory.omegaPlan(:);
    U0=[old(2:end);old(end)]-wr;
end
U0=min(max(U0,lb),ub);
%% 3. Receding-horizon optimization; no fallback controller is hidden
opts=optimoptions('fmincon','Algorithm','sqp','Display','none', ...
    'MaxIterations',p.maxIterations,'ConstraintTolerance',p.tolerance, ...
    'OptimalityTolerance',p.tolerance, ...
    'SpecifyObjectiveGradient',false);
clock=tic;
try
    [U,fval,flag,output]=fmincon(@(z)mpc_objective(z,e0,p),U0, ...
        [],[],[],[],lb,ub,@tracking_mpc_constraints,opts);
catch ME
    info.message=ME.message; info.solveSeconds=toc(clock); return
end
info.solveSeconds=toc(clock); info.exitflag=flag; info.objective=fval;
if numel(U)~=p.N || any(~isfinite(U(:))) || ~isfinite(fval)
    info.message='Solver returned an invalid candidate.'; return
end
U=U(:);
res=max([0;lb-U;U-ub]);
[~,~,E]=mpc_objective(U,e0,p);
info.constraintResidual=res; info.predictedError=E;
info.ok=flag>0 && res<=10*p.tolerance && all(isfinite(E(:)));
if ~info.ok
    info.message=sprintf('MPC rejected: exitflag=%d, bound residual=%.3g. %s',flag,res,output.message);
    return
end
%% 4. Execute only the first physical angular rate and retain a warm start
omega=wr(1)+U(1);
memory=struct('omegaPlan',wr+U,'inputPlan',U,'lastObjective',fval);
info.message=sprintf('%s MPC accepted, exitflag=%d.',p.mode,flag);
end
