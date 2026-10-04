function result = run_simulation(path,c,callback,engine)
% RUN_SIMULATION Execute Simulink or the explicitly selected MATLAB verification loop.
if nargin<3, callback=[]; end
if nargin<4, engine='simulink'; end
% Run length is determined by the desired trajectory, never a time override.
if isfield(c,'duration'), c=rmfield(c,'duration'); end
c.desiredDuration=path.length/c.v0;
validateattributes(c.desiredDuration,{'double'},{'scalar','real','finite','positive'});
c=prepare_controller_design(path,c);
validate_simulation_config(c);
if isfield(c,'skipPathValidation') && c.skipPathValidation
    check=summarize_trajectory(path,c);
else
    check=check_trajectory(path,c);
    assert(check.ok,'Trajectory:PathRejected','Path check failed. Adjust the path or limits first.');
end
s=SimulationSession(path,c,callback);
stopTime=c.desiredDuration;
if strcmp(engine,'matlab')
    q=c.q0; previous=0;
    for t=0:c.Ts:stopTime
        [hat,~]=s.observe(t,q,previous);
        w=s.control(t,hat);
        if s.StopRequested, break; end
        q=s.advance(q,w); previous=w;
    end
elseif strcmp(engine,'simulink')
    try
        file=build_simulation_model;
        [~,mdl]=fileparts(file);
        load_system(file);
        assert(strcmp(get_param(mdl,'SimulationStatus'),'stopped'),'Trajectory:Running','The model is already running.');
        assert(strcmp(get_param(mdl,'Dirty'),'off'),'Trajectory:Unsaved','Save or discard manual model edits before running.');
        key=simulation_registry('register',[],s);
        cleanup=onCleanup(@() simulation_registry('remove',key));
        in=Simulink.SimulationInput(mdl);
        % Override block expressions with numeric literals, with no workspace lookup.
        blocks={'Robot plant','Sensor and estimator','Reference and controller'};
        for j=1:numel(blocks)
            in=in.setBlockParameter([mdl '/' blocks{j}],'Parameters',num2str(key,17));
        end
        in=in.setBlockParameter([mdl '/Previous applied omega'],'SampleTime',num2str(c.Ts,17));
        in=in.setModelParameter('FixedStep',num2str(c.Ts,17),'StopTime',num2str(stopTime,17), ...
            'SimulationMode','normal','FastRestart','off');
        sim(in);
    catch ME
        s.Status='failed';
        details=getReport(ME,'extended','hyperlinks','off');
        s.Message=regexprep(details,'<a[^>]*>(.*?)</a>','$1');
        fprintf(2,'Trajectory Simulink failure:\n%s\n',s.Message);
    end
else
    error('Trajectory:Engine','Unknown simulation engine.');
end
if strcmp(s.Status,'running')
    s.Status='completed'; s.Message='Desired duration reached. Simulation completed.';
end
result=struct('config',c,'path',path,'check',check,'log',s.Log, ...
    'status',s.Status,'message',s.Message,'engine',engine,'created',char(datetime('now')));
result.metrics=tracking_metrics(result);
end
