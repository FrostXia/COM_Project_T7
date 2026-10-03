function [ehat,m]=kf_error(y,previous,c,m)
% KF_ERROR Optional time-varying error-state Kalman filter (not a pose EKF).
if strcmp(c.filter.mode,'none'), ehat=y; m=[]; return; end
assert(~isempty(c.filter.Qn) && ~isempty(c.filter.Rn), ...
    'P7:KalmanCovarianceUnset', ...
    'Fill c.filter.Qn (scalar) and c.filter.Rn (3-by-3) before enabling KF.');
%% 1. Prediction from the previous APPLIED command, not unknown actuator noise
if isempty(m)
    m.mean=c.filter.initialMean; m.P=c.filter.P0;
else
    [Ad,Bd,dd]=p7utils.linear_step(previous.wr,c.v0,previous.vr,c.Ts);
    m.mean=Ad*m.mean+Bd*previous.u+dd;
    Qx=Bd*(c.filter.qScale*c.filter.Qn)*Bd';
    m.P=Ad*m.P*Ad'+Qx;
end
%% 2. Full error measurement update with wrapped heading innovation
Rn=c.filter.rScale*c.filter.Rn;
innovation=y-m.mean;
innovation(3)=p7_wrap(innovation(3));
L=m.P/(m.P+Rn);
m.mean=m.mean+L*innovation;
m.mean(3)=p7_wrap(m.mean(3));
J=eye(3)-L;
m.P=J*m.P*J'+L*Rn*L';
m.P=(m.P+m.P')/2;
m.L=L;
ehat=m.mean;
end
