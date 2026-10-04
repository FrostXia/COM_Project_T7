function test_nominal_turn
% TEST_NOMINAL_TURN Verify filtering, sign selection and run-level overrides.
settings=struct('thresholdDeg',1,'fallbackDeg',1);
make=@(rates)struct('s',(0:numel(rates)-1)', ...
    'kappa',deg2rad(rates(:))/0.5);
a=compute_nominal_turn(make([10 -10 10 -10]),0.5,settings);
assert(abs(a.degreesPerSecond-10)<1e-12 && a.positiveCount==2 && a.negativeCount==2);
a=compute_nominal_turn(make([10 -2 -4]),0.5,settings);
assert(abs(a.degreesPerSecond+16/3)<1e-12);
a=compute_nominal_turn(make([0 0.9 -0.9]),0.5,settings);
assert(a.usedFallback && abs(a.degreesPerSecond-1)<1e-12 && a.filteredCount==3);
a=compute_nominal_turn(make([1 -1 0.5]),0.5,settings);
assert(~a.usedFallback && a.positiveCount==1 && a.negativeCount==1);
a=compute_nominal_turn(make([-5 -3 0]),0.5,settings);
assert(abs(a.degreesPerSecond+4)<1e-12 && a.positiveCount==0);
a=compute_nominal_turn(make([5 3 0]),0.5,settings);
assert(abs(a.degreesPerSecond-4)<1e-12 && a.negativeCount==0);
bad=make([0 NaN]); rejected=false;
try
    compute_nominal_turn(bad,0.5,settings);
catch
    rejected=true;
end
assert(rejected,'Invalid data must not use the fallback.');
c=default_config; c.controller='lqr';
path=generate_trajectory('line',[12 3 .15 1],[],c);
resolved=prepare_controller_design(path,c);
assert(abs(rad2deg(resolved.design.wrNom)-1)<1e-12);
assert(all(path.kappa==0),'Nominal fallback must not change pointwise reference rates.');
c.nominalOverride=deg2rad(-7);
resolved=prepare_controller_design(path,c);
assert(abs(rad2deg(resolved.design.wrNom)+7)<1e-12 && strcmp(resolved.nominalReference.source,'manual'));
c.nominalOverride=[]; c.v0=1;
path=generate_trajectory('circle',[12 3 .15 1],[],c);
resolved=prepare_controller_design(path,c);
assert(abs(resolved.design.wrNom-1/3)<1e-12 && resolved.design.v0==1);
disp('PASS: nominal turn threshold, majority sign, fallback and manual override');
end
