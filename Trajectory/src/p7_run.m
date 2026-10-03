function result = p7_run(path,c,callback,engine)
% P7_RUN Execute Simulink or the explicitly selected MATLAB verification loop.
if nargin<3, callback=[]; end
if nargin<4, engine='simulink'; end
p7_validate_config(c);
check=p7_check_path(path,c);
assert(check.ok,'P7:PathRejected','Path check failed. Adjust the path or limits first.');
s=P7Session(path,c,callback);
stopTime=floor(c.duration/c.Ts)*c.Ts;
if strcmp(c.referenceMode,'time')
    stopTime=min(stopTime,floor(path.length/c.v0/c.Ts)*c.Ts);
end
assert(stopTime>=c.Ts,'P7:Duration','Reference is shorter than one sample period.');
if strcmp(engine,'matlab')
    q=c.q0; previous=0;
    for k=0:round(stopTime/c.Ts)
        [hat,~]=s.observe(k*c.Ts,q,previous);
        w=s.control(k*c.Ts,hat);
        if s.StopRequested, break; end
        q=s.advance(q,w); previous=w;
    end
elseif strcmp(engine,'simulink')
    file=build_p7_model;
    [~,mdl]=fileparts(file);
    load_system(file);
    assert(strcmp(get_param(mdl,'SimulationStatus'),'stopped'),'P7:Running','The model is already running.');
    assert(strcmp(get_param(mdl,'Dirty'),'off'),'P7:Unsaved','Save or discard manual model edits before running.');
    [~,key]=fileparts(tempname);
    p7_registry('set',key,s);
    cleanup=onCleanup(@() p7_registry('remove',key));
    in=Simulink.SimulationInput(mdl);
    in=in.setVariable('p7SessionKey',key);
    in=in.setVariable('p7Ts',c.Ts);
    in=in.setModelParameter('FixedStep',num2str(c.Ts,17),'StopTime',num2str(stopTime,17), ...
        'SimulationMode','normal','FastRestart','off');
    try
        sim(in);
    catch ME
        s.Status='failed'; s.Message=ME.message;
    end
else
    error('P7:Engine','Unknown simulation engine.');
end
if strcmp(s.Status,'running')
    s.Status='completed'; s.Message='Configured simulation horizon completed.';
end
result=struct('config',c,'path',path,'check',check,'log',s.Log, ...
    'status',s.Status,'message',s.Message,'engine',engine,'created',char(datetime('now')));
result.metrics=p7_metrics(result);
end
