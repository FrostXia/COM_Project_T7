function [K,Klqr,design] = design_feedback_gains(p,verbose,parametersOnly)
% DESIGN_FEEDBACK_GAINS Q4-style continuous single-input Trajectory design.
% [K,Klqr,design] = design_feedback_gains
% Optional p overrides are for controlled parameter experiments.
%% 1. Screenshot parameters and explicit override validation
base=struct('v0',0.5,'wrNom',pi/4,'poles',[-1 -2 -4],'Q',eye(3),'R',1);
if nargin<1 || isempty(p), p=base; else
    names=fieldnames(p);
    assert(all(ismember(names,fieldnames(base))),'Trajectory:Parameter','Unknown design field.');
    for i=1:numel(names), base.(names{i})=p.(names{i}); end
    p=base;
end
if nargin<2, verbose=true; end
if nargin<3, parametersOnly=false; end
validateattributes(p.v0,{'double'},{'scalar','real','finite','positive'});
validateattributes(p.wrNom,{'double'},{'scalar','real','finite','nonzero'});
validateattributes(p.poles,{'double'},{'vector','numel',3,'finite'});
assert(all(real(p.poles)<0),'Trajectory:Poles','Continuous poles must have negative real parts.');
assert(isequal(size(p.Q),[3 3]) && isreal(p.Q) && all(isfinite(p.Q(:))) && ...
    norm(p.Q-p.Q','fro')<1e-10 && min(eig(p.Q))>0,'Trajectory:Q','This prototype requires Q positive definite.');
validateattributes(p.R,{'double'},{'scalar','real','finite','positive'});
%% 2. Nonlinear error convention u = omega - omegaReference
A=[0 p.wrNom 0;-p.wrNom 0 p.v0;0 0 0];
B=[0;0;-1];
Co=[B A*B A*A*B];
assert(rank(Co)==3,'Trajectory:Controllability','Nominal model is not fully controllable.');
design=struct('parameters',p,'A',A,'B',B,'controllability',Co);
K=[]; Klqr=[];
if parametersOnly, return; end
assert(exist('place','file')~=0 && exist('lqr','file')~=0, ...
    'Trajectory:Toolbox','Control System Toolbox is required for place and lqr.');
%% 3. Pole placement, as in Assignment 2 Q4
K=place(A,B,p.poles);
Acl=A-B*K;
design.K=K; design.Acl=Acl; design.polesSF=eig(Acl);
target=poly(p.poles); actual=poly(Acl);
assert(norm(actual-target)<1e-7*max(1,norm(target)), ...
    'Trajectory:Verification','Pole placement verification failed.');
%% 4. Continuous LQR and Riccati residual
[Klqr,Plqr,Elqr]=lqr(A,B,p.Q,p.R);
Acl_lqr=A-B*Klqr;
design.Klqr=Klqr; design.Plqr=Plqr;
design.Acl_lqr=Acl_lqr; design.polesLQR=Elqr;
design.careResidual=norm(A'*Plqr+Plqr*A-Plqr*B*(p.R\(B'*Plqr))+p.Q,'fro');
assert(all(real(eig(Acl_lqr))<0),'Trajectory:Verification','Nominal LQR is not stable.');
%% 5. Print matrices and poles; keep full precision in returned variables
if verbose
    fprintf('\nTrajectory: v0=%.6g m/s, nominal Wr=%.6g rad/s\n',p.v0,p.wrNom);
    disp('A ='); disp(A); disp('B ='); disp(B);
    disp('K ='); disp(K); disp('Acl = A-B*K ='); disp(Acl);
    disp('eig(Acl) ='); disp(design.polesSF);
    disp('Q ='); disp(p.Q); disp('R ='); disp(p.R);
    disp('Klqr ='); disp(Klqr); disp('Plqr ='); disp(Plqr);
    disp('Acl_lqr = A-B*Klqr ='); disp(Acl_lqr);
    disp('eig(Acl_lqr) ='); disp(Elqr);
    fprintf('CARE residual: %.3g\n',design.careResidual);
end
end
