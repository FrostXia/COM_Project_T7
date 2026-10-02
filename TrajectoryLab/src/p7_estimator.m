function [qHat,m] = p7_estimator(y,previousOmega,c,m)
% P7_ESTIMATOR Model prediction with optional EKF measurement correction.
% No ground-truth argument is available to this function.
if strcmp(c.estimatorMode,'external')
    assert(~isempty(c.estimatorFcn),'P7:Estimator','Set config.estimatorFcn.');
    [qHat,m] = c.estimatorFcn(y,previousOmega,c,m);
    return
end
if isempty(m)
    m.q = c.qHat0;
    m.P = c.P0;
else
    [m.q,F] = p7_motion(m.q,previousOmega,c);
    W = diag((c.stateStd*double(c.stateNoiseEnabled)).^2);
    m.P = F*m.P*F'+W;
end
if strcmp(c.estimatorMode,'ekf')
    ny = numel(y);
    H = eye(3); H = H(1:ny,:);
    sd = c.measurementStd(1:ny)*double(c.measurementNoiseEnabled);
    R = diag(max(sd.^2,c.measurementFloor));
    innovation = y-H*m.q;
    if ny==3, innovation(3)=p7_wrap(innovation(3)); end
    K = (m.P*H')/(H*m.P*H'+R);
    m.q = m.q+K*innovation;
    A = eye(3)-K*H;
    m.P = A*m.P*A'+K*R*K';
    m.P = (m.P+m.P')/2;
end
qHat = m.q;
end
