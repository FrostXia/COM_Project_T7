function [qHat,m] = estimate_robot_pose(y,previousOmega,c,m)
% ESTIMATE_ROBOT_POSE Model prediction with optional EKF measurement correction.
% No ground-truth argument is available to this function.
if strcmp(c.estimatorMode,'external')
    assert(~isempty(c.estimatorFcn),'Trajectory:Estimator','Set config.estimatorFcn.');
    [qHat,m] = c.estimatorFcn(y,previousOmega,c,m);
    return
end
if isempty(m)
    m.q = c.qHat0;
    m.P = c.P0;
else
    previousPose=m.q;
    [m.q,F] = integrate_robot_motion(previousPose,previousOmega,c);
    W = diag((c.stateStd*double(c.stateNoiseEnabled)).^2);
    if isfield(c,'controlNoiseEnabled') && c.controlNoiseEnabled
        % Local input-noise covariance; saturation makes this an approximation.
        delta=1e-5;
        G=(integrate_robot_motion(previousPose,previousOmega+delta,c)- ...
            integrate_robot_motion(previousPose,previousOmega-delta,c))/(2*delta);
        W=W+c.controlStd^2*(G*G');
    end
    if isfield(c,'estimatorQn') && ~isempty(c.estimatorQn), W=c.estimatorQn; end
    m.P = F*m.P*F'+W;
end
if strcmp(c.estimatorMode,'ekf')
    ny = numel(y);
    H = eye(3); H = H(1:ny,:);
    sd = c.measurementStd(1:ny)*double(c.measurementNoiseEnabled);
    R = diag(max(sd.^2,c.measurementFloor));
    if isfield(c,'estimatorRn') && ~isempty(c.estimatorRn)
        R=c.estimatorRn(1:ny,1:ny)+c.measurementFloor*eye(ny);
    end
    innovation = y-H*m.q;
    if ny==3, innovation(3)=wrap_angle(innovation(3)); end
    K = (m.P*H')/(H*m.P*H'+R);
    m.q = m.q+K*innovation;
    A = eye(3)-K*H;
    m.P = A*m.P*A'+K*R*K';
    m.P = (m.P+m.P')/2;
end
qHat = m.q;
end
