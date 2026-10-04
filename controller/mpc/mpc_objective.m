function [f,g,E]=mpc_objective(U,e0,p)
% MPC_OBJECTIVE Roll out states internally; optimize controls only.
U=U(:); E=zeros(3,p.N+1); E(:,1)=e0;
f=0; g=[]; % The nonlinear solver uses finite-difference gradients.
%% 1. Tracking and correction-input cost over the prediction horizon
for j=1:p.N
    E(:,j+1)=predict_tracking_error(E(:,j),U(j),p,j);
    f=f+p.Ts*(E(:,j+1)'*p.Q*E(:,j+1)+p.R*U(j)^2);
end
%% 2. Optional terminal cost; default is zero, not a stability certificate
f=f+E(:,end)'*p.terminalWeight*E(:,end);
end
