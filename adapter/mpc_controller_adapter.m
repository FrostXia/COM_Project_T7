function [omega,memory,info]=mpc_controller_adapter(qHat,ref,config,memory)
% MPC_CONTROLLER_ADAPTER Map the platform pose interface to nonlinear error MPC.
%% 1. Map public platform configuration into the shared MPC configuration
c=config; c.vr=config.v0;
defaults=default_config; c.mpc=defaults.algorithm;
c.Ts=config.Ts; c.N=config.N;
c.wMin=config.wMin; c.wMax=config.wMax;
if isfield(config,'algorithm')
    names=fieldnames(config.algorithm);
    for j=1:numel(names)
        assert(isfield(c.mpc,names{j}),'Trajectory:Algorithm','Unknown MPC option.');
        c.mpc.(names{j})=config.algorithm.(names{j});
    end
end
validateattributes(qHat,{'double'},{'size',[3 1],'finite','real'});
assert(isequal(size(ref.q),[3 c.N+1]),'Trajectory:Preview','ref.q must be 3-by-(N+1).');
%% 2. Convert estimated global pose to body-frame tracking error
e0=tracking_error_state(qHat,ref.q(:,1));
vr=c.vr*ones(c.N,1);
if isfield(ref,'vr'), vr=ref.vr; end
[omega,memory,info]=solve_nonlinear_mpc(e0,ref.w,vr,c,memory);
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
