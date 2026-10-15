function next=predict_tracking_error(e,u,p,j)
% PREDICT_TRACKING_ERROR Predict the ERROR state, not global pose.
% Integrate the nonlinear error dynamics with RK4.
w=p.wr(j)+u; vr=p.vr(j); v0=p.v0;
f=@(x)[w*x(2)-v0+vr*cos(x(3));-w*x(1)+vr*sin(x(3));-u];
h=p.Ts/p.predictionSubsteps; next=e;
for k=1:p.predictionSubsteps
    a=f(next); b=f(next+h*a/2); cc=f(next+h*b/2); d=f(next+h*cc);
    next=next+h*(a+2*b+2*cc+d)/6;
end
end
