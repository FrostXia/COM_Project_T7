function next=predict_error_Project7(e,u,p,j)
% PREDICT_ERROR_PROJECT7 Predict the ERROR state, not global pose.
%% 1. Frozen-interval LTV prediction
if strcmp(p.mode,'ltv')
    next=p.Ad(:,:,j)*e+p.Bd(:,j)*u+p.dd(:,j);
    return
end
%% 2. Optional nonlinear error model, integrated with RK4
w=p.wr(j)+u; vr=p.vr(j); v0=p.v0;
f=@(x)[w*x(2)-v0+vr*cos(x(3));-w*x(1)+vr*sin(x(3));-u];
h=p.Ts/p.predictionSubsteps; next=e;
for k=1:p.predictionSubsteps
    a=f(next); b=f(next+h*a/2); cc=f(next+h*b/2); d=f(next+h*cc);
    next=next+h*(a+2*b+2*cc+d)/6;
end
end
