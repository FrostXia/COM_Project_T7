function stats=compute_nominal_turn(path,v0,settings)
% COMPUTE_NOMINAL_TURN Filter small turns and select a signed representative rate.
% The result is a fixed design value, never a replacement for reference rates.
if nargin<3, settings=struct('thresholdDeg',1,'fallbackDeg',1); end
validateattributes(v0,{'double'},{'scalar','real','finite','positive'});
validateattributes(settings.thresholdDeg,{'double'},{'scalar','real','finite','positive'});
validateattributes(settings.fallbackDeg,{'double'},{'scalar','real','finite','positive'});
assert(isstruct(path) && all(isfield(path,{'s','kappa'})),'Trajectory:NominalPath','Missing path geometry.');
validateattributes(path.s,{'double'},{'vector','real','finite','nonempty'});
validateattributes(path.kappa,{'double'},{'vector','real','finite','nonempty'});
s=path.s(:); kappa=path.kappa(:);
assert(numel(s)==numel(kappa) && numel(s)>=2 && all(diff(s)>0), ...
    'Trajectory:NominalPath','Path must have matching samples and increasing arc length.');
% Equal-arc samples have equal duration at constant speed.
uniform=linspace(s(1),s(end),numel(s))';
rates=v0*interp1(s,kappa,uniform,'linear');
threshold=deg2rad(settings.thresholdDeg);
% Include exact-threshold samples despite harmless floating-point rounding.
positive=rates>=threshold-8*eps(threshold);
negative=rates<=-threshold+8*eps(threshold);
np=sum(positive); nn=sum(negative);
mp=NaN; mn=NaN;
if np>0, mp=mean(rates(positive)); end
if nn>0, mn=mean(rates(negative)); end
fallback=np+nn==0;
if fallback
    nominal=deg2rad(settings.fallbackDeg);
else
    magnitude=(sum(abs(rates(positive)))+sum(abs(rates(negative))))/(np+nn);
    nominal=magnitude;
    if nn>np, nominal=-nominal; end
end
stats=struct('radiansPerSecond',nominal,'degreesPerSecond',rad2deg(nominal), ...
    'positiveCount',np,'negativeCount',nn,'positiveMean',mp,'negativeMean',mn, ...
    'sampleCount',numel(rates),'filteredCount',numel(rates)-np-nn, ...
    'usedFallback',fallback,'thresholdDeg',settings.thresholdDeg, ...
    'fallbackDeg',settings.fallbackDeg,'method','filtered-absolute-mean-majority-sign');
end
