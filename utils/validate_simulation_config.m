function validate_simulation_config(c)
% VALIDATE_SIMULATION_CONFIG Reject ambiguous or invalid simulation settings.
validateattributes(c.v0,{'double'},{'scalar','real','finite','positive'});
validateattributes(c.Ts,{'double'},{'scalar','real','finite','positive'});
validateattributes(c.N,{'double'},{'scalar','integer','>=',1,'<=',1000});
validateattributes(c.q0,{'double'},{'size',[3,1],'real','finite'});
validateattributes(c.initialEstimateOffset,{'double'},{'size',[3,1],'real','finite'});
validateattributes(c.stateStd,{'double'},{'size',[3,1],'finite','nonnegative'});
validateattributes(c.measurementStd,{'double'},{'size',[3,1],'finite','nonnegative'});
validateattributes(c.seed,{'double'},{'scalar','integer','>=',0,'<',2^32-2});
validateattributes(c.endpointTolerance,{'double'},{'scalar','finite','positive'});
validateattributes(c.spawnFraction,{'double'},{'scalar','finite','nonnegative'});
validateattributes(c.controlStd,{'double'},{'scalar','real','finite','nonnegative'});
for key={'controlNoiseEnabled','measurementNoiseEnabled','stateNoiseEnabled'}
    validateattributes(c.(key{1}),{'logical','double'},{'scalar','binary'});
end
for key={'estimatorQn','estimatorRn'}
    matrix=c.(key{1});
    if ~isempty(matrix)
        assert(isequal(size(matrix),[3 3]) && isreal(matrix) && all(isfinite(matrix(:))) && ...
            norm(matrix-matrix','fro')<1e-10 && min(eig(matrix))>=0, ...
            'Trajectory:Covariance','Invalid code-configured covariance %s.',key{1});
    end
end
validateattributes(c.lookahead,{'double'},{'scalar','finite','positive'});
validateattributes(c.pathSpacing,{'double'},{'scalar','finite','positive'});
if isfield(c,'maxPathDeviation')
    validateattributes(c.maxPathDeviation,{'double'},{'scalar','finite','nonnegative'});
end
assert(isfinite(c.wMin) && isfinite(c.wMax) && c.wMin < 0 && c.wMax > 0, ...
    'Trajectory:Bounds','Turning bounds must straddle zero.');
assert(all(size(c.P0)==[3 3]) && all(isfinite(c.P0(:))) && ...
    norm(c.P0-c.P0','fro')<1e-10 && min(eig(c.P0))>=0,'Trajectory:P0','Invalid initial covariance.');
assert(ismember(c.referenceMode,{'path','time'}),'Trajectory:Mode','Invalid reference mode.');
assert(ismember(c.estimatorMode,{'ekf','deadreckoning','ideal','external'}),'Trajectory:Estimator','Invalid estimator mode.');
assert(ismember(c.sensorMode,{'position','pose'}),'Trajectory:Sensor','Invalid sensor mode.');
if isfield(c,'nominalOverride') && ~isempty(c.nominalOverride)
    validateattributes(c.nominalOverride,{'double'},{'scalar','real','finite','nonzero'});
end
assert(ismember(c.controller,{'openloop','sf','lqr','mpc','external'}),'Trajectory:Controller','Invalid controller.');
end
