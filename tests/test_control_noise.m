function test_control_noise
% TEST_CONTROL_NOISE Verify hidden execution noise, clipping and reproducibility.
root=fileparts(fileparts(mfilename('fullpath'))); addpath(root); setup_paths;
c=default_config; c.referenceMode='time'; c.controller='openloop';
c.preset.line.length=.5;
c.q0=[0;0;0]; c.initialEstimateOffset=zeros(3,1);
c.estimatorMode='deadreckoning'; c.measurementNoiseEnabled=false; c.stateNoiseEnabled=false;
path=generate_trajectory('line',c.preset,[],c);
c.controlNoiseEnabled=false;
clean=run_simulation(path,c,[],'matlab');
assert(norm(clean.log.qTrue-clean.log.qHat,'fro')<1e-10);
assert(all(clean.log.controlNoise==0));
c.controlNoiseEnabled=true; c.controlStd=4; c.wMax=.2; c.wMin=-.2;
one=run_simulation(path,c,[],'matlab'); two=run_simulation(path,c,[],'matlab');
assert(isequal(one.log.qTrue,two.log.qTrue));
assert(isequal(one.log.controlNoise,two.log.controlNoise));
assert(any(abs(one.log.controlNoise)>c.wMax));
assert(all(abs(one.log.omegaPhysical)<=c.wMax+1e-12));
assert(all(one.log.omega==0) && norm(one.log.qTrue-one.log.qHat,'fro')>1e-6);
assert(max(abs(one.log.omegaPhysical-min(max(one.log.omega+one.log.controlNoise,c.wMin),c.wMax)))<1e-12);
assert(max(abs(one.log.controlNoiseEffective-(one.log.omegaPhysical-one.log.omega)))<1e-12);
% Measurement noise must not consume or change actuator random samples.
c.measurementNoiseEnabled=true; observed=run_simulation(path,c,[],'matlab');
assert(isequal(observed.log.controlNoise,one.log.controlNoise));
assert(isequal(observed.log.qTrue,one.log.qTrue));
% A saturated command is perturbed before the final physical-input limiter.
c.skipPathValidation=true; circle=generate_trajectory('circle',[12 .1 .15 1],[],c);
saturated=run_simulation(circle,c,[],'matlab');
assert(all(saturated.log.omega==c.wMax));
assert(any(saturated.log.omegaPhysical<c.wMax));
% Export the complete desired path as re-importable XY and independent evaluation.
destination=tempname; mkdir(destination); cleanup=onCleanup(@() rmdir(destination,'s'));
folder=export_experiment(one,destination);
paths=dir(fullfile(folder,'trajectory_*.csv')); evaluations=dir(fullfile(folder,'evaluation_*.csv'));
assert(isscalar(paths) && isscalar(evaluations));
assert(strcmp(erase(paths.name,'trajectory_'),erase(evaluations.name,'evaluation_')));
xy=readmatrix(fullfile(folder,paths.name));
assert(size(xy,2)==2 && max(abs(xy-one.path.xy),[],'all')<1e-10);
lines=splitlines(string(fileread(fullfile(folder,evaluations.name))));
assert(strtrim(lines(1))=="Metric,Value,Unit");
options=delimitedTextImportOptions('NumVariables',3,'Delimiter',',', ...
    'DataLines',[2 Inf],'VariableNames',{'Metric','Value','Unit'}, ...
    'VariableTypes',{'string','string','string'});
evaluation=readtable(fullfile(folder,evaluations.name),options);
assert(isequal(evaluation.Properties.VariableNames,{'Metric','Value','Unit'}));
id=evaluation.Metric=="Position Root Mean Square Error";
expected=sqrt(mean(sum((one.log.qTrue(1:2,:)-one.log.reference(1:2,:)).^2,1)));
assert(abs(str2double(evaluation.Value(id))-expected)<1e-12);
id=evaluation.Metric=="Maximum Absolute Angular Velocity";
assert(abs(str2double(evaluation.Value(id))-max(abs(one.log.omegaPhysical)))<1e-12);
assert(~any(contains(evaluation.Metric,["Settling","Tolerance","Control Effort","Endpoint Reached"])));
% Export contains the complete desired path even when the robot falls behind.
assert(isequal(size(xy),size(one.path.xy)) && xy(end,1)>one.log.qTrue(1,end));
export_experiment(two,destination);
assert(numel(dir(fullfile(folder,'trajectory_*.csv')))==2);
assert(numel(dir(fullfile(folder,'evaluation_*.csv')))==2);
% Geometric distance uses segments rather than only sampled vertices.
probe=one; probe.path.xy=[0 0;2 0]; probe.path.length=2;
probe.log.qTrue(1:2,:)=repmat([1;1],1,numel(probe.log.t));
evaluation=evaluate_experiment(probe);
id=evaluation.Metric=="Path Distance Root Mean Square Error";
assert(abs(str2double(evaluation.Value(id))-1)<1e-12);
probe.log.t=[]; evaluation=evaluate_experiment(probe);
assert(str2double(evaluation.Value(evaluation.Metric=="Recorded Sample Count"))==0);
disp('PASS: actuator noise, final clipping, estimator boundary and export');
end
