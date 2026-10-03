function [f,g,E]=objfun_Project7_MPC(U,e0,p)
% OBJFUN_PROJECT7_MPC Roll out states internally; optimize controls only.
U=U(:); E=zeros(3,p.N+1); E(:,1)=e0;
f=0; g=zeros(p.N,1); sensitivity=zeros(3,p.N);
%% 1. Tracking and correction-input cost over the prediction horizon
for j=1:p.N
    E(:,j+1)=predict_error_Project7(E(:,j),U(j),p,j);
    f=f+p.Ts*(E(:,j+1)'*p.Q*E(:,j+1)+p.R*U(j)^2);
    if strcmp(p.mode,'ltv')
        sensitivity=p.Ad(:,:,j)*sensitivity;
        sensitivity(:,j)=sensitivity(:,j)+p.Bd(:,j);
        g=g+2*p.Ts*sensitivity'*p.Q*E(:,j+1);
        g(j)=g(j)+2*p.Ts*p.R*U(j);
    end
end
%% 2. Optional terminal cost; default is zero, not a stability certificate
f=f+E(:,end)'*p.terminalWeight*E(:,end);
if strcmp(p.mode,'ltv')
    g=g+2*sensitivity'*p.terminalWeight*E(:,end);
else
    g=[]; % fmincon finite differences are selected for the nonlinear model.
end
end
