function test_desired_duration(includeSimulink)
% TEST_DESIRED_DURATION Stop at the reference horizon without pursuit time.
if nargin<1, includeSimulink=false; end
root=fileparts(fileparts(mfilename('fullpath'))); addpath(root); setup_paths;
c=default_config; c.referenceMode='time'; c.controller='openloop';
c.measurementNoiseEnabled=false; c.controlNoiseEnabled=false; c.stateNoiseEnabled=false;
c.estimatorMode='ideal'; c.q0=[-2;0;0];
c.pathSpacing=.001;
assert(~isfield(c,'duration'));
for duration=[.25 .5 .04]
    params=c.preset; params.line.length=c.v0*duration;
    path=generate_trajectory('line',params,[],c);
    % Legacy inputs and stale derived values must not override path timing.
    c.duration=999; c.desiredDuration=888;
    a=run_simulation(path,c,[],'matlab');
    assert(strcmp(a.status,'completed') && ~isfield(a.config,'duration'));
    assert(abs(a.config.desiredDuration-duration)<1e-12);
    expected=0:c.Ts:duration;
    assert(isequal(size(a.log.t),size(expected)) && max(abs(a.log.t-expected))<1e-12);
    assert(all(a.log.t<=duration+1e-12) && duration-a.log.t(end)<c.Ts+1e-12);
    assert(~a.metrics.endpointReached && ~isfield(a.metrics,'extraCompletionTime'));
    evaluation=evaluate_experiment(a);
    assert(sum(evaluation.Metric=="Desired Duration")==1);
    assert(~any(contains(evaluation.Metric,"Maximum Duration")));
    if includeSimulink
        b=run_simulation(path,c,[],'simulink');
        assert(strcmp(b.status,'completed'),'Trajectory:Simulink','%s',b.message);
        assert(isequal(size(a.log.t),size(b.log.t)) && max(abs(a.log.t-b.log.t))<1e-12);
        assert(max(abs(a.log.qTrue-b.log.qTrue),[],'all')<1e-12);
    end
end
disp('PASS: automatic desired duration, fractional sample bounds and no endpoint pursuit');
end
