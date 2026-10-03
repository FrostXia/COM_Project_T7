function validate(c)
% VALIDATE Check shared simulation inputs before a run.
validateattributes(c.v0,{'double'},{'scalar','finite','positive'});
validateattributes(c.vr,{'double'},{'scalar','finite','positive'});
validateattributes(c.Ts,{'double'},{'scalar','finite','positive'});
validateattributes(c.N,{'double'},{'scalar','integer','positive'});
validateattributes(c.duration,{'double'},{'scalar','finite','>=',c.Ts});
validateattributes(c.integrationSteps,{'double'},{'scalar','integer','positive'});
validateattributes(c.q0,{'double'},{'size',[3 1],'finite','real'});
validateattributes(c.qr0,{'double'},{'size',[3 1],'finite','real'});
validateattributes(c.measurementStd,{'double'},{'size',[3 1],'finite','nonnegative'});
validateattributes(c.controlStd,{'double'},{'scalar','finite','nonnegative'});
validateattributes(c.seed,{'double'},{'scalar','integer','>=',0,'<',2^32-2});
assert(isfinite(c.wMin) && isfinite(c.wMax) && c.wMin<0 && c.wMax>0,'P7:Bounds','Bounds must straddle zero.');
assert(ismember(c.filter.mode,{'none','kf'}),'P7:Filter','Filter must be none or kf.');
assert(ismember(c.mpc.mode,{'ltv','nonlinear'}),'P7:MPC','Unknown MPC model.');
validateattributes(c.analysisStart,{'double'},{'scalar','finite','nonnegative'});
if strcmp(c.filter.mode,'kf')
    assert(~isempty(c.filter.Qn) && ~isempty(c.filter.Rn), ...
        'P7:KalmanCovarianceUnset', ...
        'Fill c.filter.Qn (scalar) and c.filter.Rn (3-by-3) before enabling KF.');
    validateattributes(c.filter.Qn,{'double'},{'scalar','real','finite','nonnegative'});
    validateattributes(c.filter.qScale,{'double'},{'scalar','real','finite','positive'});
    validateattributes(c.filter.rScale,{'double'},{'scalar','real','finite','positive'});
    validateattributes(c.filter.initialMean,{'double'},{'size',[3 1],'finite','real'});
    for name={'Rn','P0'}
        P=c.filter.(name{1});
        assert(isequal(size(P),[3 3]) && isreal(P) && all(isfinite(P(:))) && ...
            norm(P-P','fro')<1e-10 && min(eig(P))>=0,'P7:Covariance','Invalid covariance.');
    end
    assert(min(eig(c.filter.Rn))>0,'P7:Covariance','Measurement covariance must be positive definite.');
end
end
