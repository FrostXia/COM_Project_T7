function p7_validate_config(c)
% P7_VALIDATE_CONFIG Reject ambiguous or invalid simulation settings.
validateattributes(c.v0,{'double'},{'scalar','real','finite','positive'});
validateattributes(c.Ts,{'double'},{'scalar','real','finite','positive'});
validateattributes(c.N,{'double'},{'scalar','integer','>=',1,'<=',1000});
validateattributes(c.duration,{'double'},{'scalar','finite','>=',c.Ts});
validateattributes(c.q0,{'double'},{'size',[3,1],'real','finite'});
validateattributes(c.initialEstimateOffset,{'double'},{'size',[3,1],'real','finite'});
validateattributes(c.stateStd,{'double'},{'size',[3,1],'finite','nonnegative'});
validateattributes(c.measurementStd,{'double'},{'size',[3,1],'finite','nonnegative'});
validateattributes(c.seed,{'double'},{'scalar','integer','>=',0,'<',2^32-2});
validateattributes(c.lookahead,{'double'},{'scalar','finite','positive'});
validateattributes(c.pathSpacing,{'double'},{'scalar','finite','positive'});
if isfield(c,'maxPathDeviation')
    validateattributes(c.maxPathDeviation,{'double'},{'scalar','finite','nonnegative'});
end
assert(isfinite(c.wMin) && isfinite(c.wMax) && c.wMin < 0 && c.wMax > 0, ...
    'P7:Bounds','Turning bounds must straddle zero.');
assert(all(size(c.P0)==[3 3]) && all(isfinite(c.P0(:))) && ...
    norm(c.P0-c.P0','fro')<1e-10 && min(eig(c.P0))>=0,'P7:P0','Invalid initial covariance.');
assert(ismember(c.referenceMode,{'path','time'}),'P7:Mode','Invalid reference mode.');
assert(ismember(c.estimatorMode,{'ekf','deadreckoning','ideal','external'}),'P7:Estimator','Invalid estimator mode.');
assert(ismember(c.sensorMode,{'position','pose'}),'P7:Sensor','Invalid sensor mode.');
assert(ismember(c.controller,{'demo','openloop','sf','lqr','mpc','external'}),'P7:Controller','Invalid controller.');
end
