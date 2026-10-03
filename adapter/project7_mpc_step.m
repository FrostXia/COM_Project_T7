function [omega,memory,info]=project7_mpc_step(qHat,ref,config,memory)
% PROJECT7_MPC_STEP TrajectoryLab's global-pose controller interface.
%% 1. Map public platform configuration into the shared MPC configuration
c=p7utils.defaults;
c.v0=config.v0; c.vr=config.v0;
c.Ts=config.Ts; c.N=config.N;
c.wMin=config.wMin; c.wMax=config.wMax;
if isfield(config,'algorithm')
    names=fieldnames(config.algorithm);
    for j=1:numel(names)
        assert(isfield(c.mpc,names{j}),'P7:Algorithm','Unknown MPC option.');
        c.mpc.(names{j})=config.algorithm.(names{j});
    end
end
validateattributes(qHat,{'double'},{'size',[3 1],'finite','real'});
assert(isequal(size(ref.q),[3 c.N+1]),'P7:Preview','ref.q must be 3-by-(N+1).');
%% 2. Convert estimated global pose to body-frame tracking error
e0=p7utils.error_state(qHat,ref.q(:,1));
[omega,memory,info]=p7_mpc_error_step(e0,ref.w,c.vr*ones(c.N,1),c,memory);
%% 3. Optional predicted global pose for the platform's diagnostics
info.predictedQ=[];
if info.ok
    E=info.predictedError; qr=ref.q;
    th=qr(3,:)-E(3,:);
    xx=qr(1,:)-cos(th).*E(1,:)+sin(th).*E(2,:);
    yy=qr(2,:)-sin(th).*E(1,:)-cos(th).*E(2,:);
    info.predictedQ=[xx;yy;th];
end
end
