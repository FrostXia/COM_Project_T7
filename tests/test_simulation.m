function summary = test_simulation(includeSimulink,includeUI)
% TEST_SIMULATION Verify geometry, estimation, reproducibility and the model harness.
if nargin<1, includeSimulink=true; end
if nargin<2, includeUI=false; end
root=fileparts(fileparts(mfilename('fullpath')));
addpath(root); setup_paths;
names={};
c=default_config;
q=[1;2;0.4]; [actual,F]=integrate_robot_motion(q,0,c);
assert(norm(actual-q-[c.v0*c.Ts*cos(q(3));c.v0*c.Ts*sin(q(3));0])<1e-12);
delta=1e-6; qp=q; qp(3)=qp(3)+delta; qm=q; qm(3)=qm(3)-delta;
assert(norm((integrate_robot_motion(qp,0,c)-integrate_robot_motion(qm,0,c))/(2*delta)-F(:,3))<1e-8);
cTurn=c; cTurn.Ts=2*pi;
assert(norm(integrate_robot_motion([0;0;0],1,cTurn)-[0;0;2*pi])<1e-12);
names{end+1}='Exact plant integration and Jacobian';

% Named defaults keep each shape independent; a circle is always one turn.
assert(~isfield(c,'presetParams') && ~isfield(c.preset.circle,'turns'));
defaultLine=generate_trajectory('line',c.preset,[],c);
assert(abs(defaultLine.length-30)<1e-10);
defaultCircle=generate_trajectory('circle',c.preset,[],c);
assert(max(abs(defaultCircle.kappa-.1))<1e-12);
assert(norm(defaultCircle.xy(end,:)-defaultCircle.xy(1,:))<1e-10);
assert(abs(defaultCircle.theta(end)-2*pi)<1e-12);
defaultSpiral=generate_trajectory('spiral',c.preset,[],c);
assert(norm(defaultSpiral.xy(end,:)-[.15*10*pi,0])<1e-10);
changed=c.preset; changed.circle.radius=2;
independentLine=generate_trajectory('line',changed,[],c);
assert(isequal(independentLine.xy,defaultLine.xy));
names{end+1}='Named preset defaults and independent shape parameters';

line=generate_trajectory('line',[12 3 .15 1],[],c); report=check_trajectory(line,c);
assert(report.ok && max(abs(line.kappa))==0);
circle=generate_trajectory('circle',[12 3 .15 1],[],c);
assert(max(abs(circle.kappa-1/3))<1e-12);
small=generate_trajectory('circle',[12 .1 .15 1],[],c);
report=check_trajectory(small,c); assert(~report.ok);
permissive=c; permissive.enforcePathTurnLimit=false;
report=check_trajectory(small,permissive);
assert(report.ok && ~report.turnFeasible && ~isempty(report.warnings));
unvalidated=c; unvalidated.skipPathValidation=true;
unvalidated.controller='openloop';
trial=run_simulation(small,unvalidated,[],'matlab');
assert(~strcmp(trial.status,'failed') && ~trial.check.checked);
assert(all(abs(trial.log.omega)<=c.wMax+1e-12));
spiral=generate_trajectory('spiral',[12 3 .15 1],[],c);
assert(all(diff(spiral.s)>0) && all(isfinite(spiral.theta)));
raw=[linspace(0,8,100)',0.2*sin(linspace(0,3,100)')];
drawn=generate_trajectory('freehand',[],raw,c);
assert(all(diff(drawn.s)>0));
names{end+1}='Preset and freehand geometry; impossible radius rejected';
test_trajectory_smoothing;
names{end+1}='Bounded adaptive smoothing: jitter removal and real bend rejection';

[qtail,wtail]=query_trajectory(line,[line.length,line.length+1],c);
assert(norm(qtail(:,2)-qtail(:,1))<1e-10 && all(wtail==0));
ct=c; ct.referenceMode='time';
[r,~]=sample_reference_preview([100;100;0],1,line,ct,[]);
assert(isequal(size(r.q),[3,c.N+1]) && isequal(size(r.w),[c.N,1]));
assert(abs(r.q(1,1)-c.v0)<1e-9);
[held,~]=sample_reference_preview([0;0;0],line.length/c.v0+1,line,ct,[]);
assert(all(held.vr==0) && all(held.w==0));
assert(max(vecnorm(held.q-held.q(:,1)))<1e-10);
ce=ct; ce.controller='openloop'; ce.estimatorMode='ideal';
ce.measurementNoiseEnabled=false; ce.stateNoiseEnabled=false; ce.controlNoiseEnabled=false;
ce.q0=[-1;0;0];
late=run_simulation(line,ce,[],'matlab');
assert(strcmp(late.status,'completed') && abs(late.log.t(end)-line.length/ce.v0)<1e-10);
assert(~late.metrics.endpointReached && ~isfield(late.metrics,'extraCompletionTime'));
assert(abs(late.metrics.positionRMSE-sqrt(mean(late.metrics.positionError.^2)))<1e-12);
ce.q0=[-1;-1;.3];
miss=run_simulation(line,ce,[],'matlab');
assert(strcmp(miss.status,'completed') && ~miss.metrics.endpointReached);
stream=RandStream('mt19937ar','Seed',123);
span=max(spiral.xy)-min(spiral.xy); headings=zeros(1,100);
for j=1:100
    spawn=sample_initial_pose(spiral,c,stream); headings(j)=spawn(3);
    assert(all(abs(spawn(1:2)-spiral.xy(1,:).')<=c.spawnFraction*span.'+1e-12));
    assert(abs(spawn(3))<=pi);
    spawn=sample_initial_pose(line,c,stream);
    assert(all(abs(spawn(1:2)-line.xy(1,:).')<=c.spawnFraction*line.length+1e-12));
end
assert(min(headings)<-pi/2 && max(headings)>pi/2);
names{end+1}='Endpoint preview, duration-only termination, full-run accuracy and bounded spawn';

ce=c; ce.qHat0=[0;0;pi-0.01]; ce.sensorMode='pose'; ce.P0=eye(3);
[hat,m]=estimate_robot_pose([0;0;-pi+0.01],0,ce,[]);
assert(abs(hat(3)-pi)<0.02 && min(eig(m.P))>=-1e-10);
cd=ce; cd.estimatorMode='deadreckoning';
[hat1,mem]=estimate_robot_pose([999;999;0],0,cd,[]);
[hat2,~]=estimate_robot_pose([-999;-999;0],0,cd,mem);
assert(norm(hat1-cd.qHat0)<1e-12 && norm(hat2-integrate_robot_motion(hat1,0,cd))<1e-12);
names{end+1}='Angular innovation and prediction-only state estimation';

c.stateNoiseEnabled=true;
line=generate_trajectory('line',[1 3 .15 1],[],c);
a=run_simulation(line,c,[],'matlab'); b=run_simulation(line,c,[],'matlab');
assert(isequal(a.log.qTrue,b.log.qTrue) && isequaln(a.log.measurement,b.log.measurement));
assert(all(isfinite(a.log.qHat(:))) && all(a.log.omega<=c.wMax+eps));
assert(norm(a.log.qTrue-a.log.qHat,'fro')>0);
names{end+1}='Seeded noisy runs and distinct estimated state';

cf=c; cf.controller='external'; cf.controllerFcn=@noTruthController;
r=run_simulation(line,cf,[],'matlab'); assert(strcmp(r.status,'completed'));
cf.controllerFcn=@unconnected_controller;
r=run_simulation(line,cf,[],'matlab'); assert(strcmp(r.status,'failed') && r.metrics.failureCount==1);
cf.controllerFcn=@badController;
r=run_simulation(line,cf,[],'matlab'); assert(strcmp(r.status,'failed'));
names{end+1}='Algorithm boundary and failed controller handling';

stopper=@(s) cancelAtSample(s,3);
r=run_simulation(line,c,stopper,'matlab'); assert(strcmp(r.status,'cancelled'));
names{end+1}='Cancellation is distinct from completed tracking';

if includeSimulink
    a=run_simulation(line,c,[],'matlab'); b=run_simulation(line,c,[],'simulink');
    assert(~strcmp(b.status,'failed'),b.message);
    assert(isequal(size(a.log.qTrue),size(b.log.qTrue)));
    assert(max(abs(a.log.qTrue(:)-b.log.qTrue(:)))<1e-10);
    assert(max(abs(a.log.qHat(:)-b.log.qHat(:)))<1e-10);
    % A second run must use its own block parameters, sample time and registry ID.
    c2=c; c2.seed=c.seed+1; c2.Ts=2*c.Ts;
    second=run_simulation(line,c2,[],'simulink');
    expected=run_simulation(line,c2,[],'matlab');
    assert(~strcmp(second.status,'failed'),second.message);
    assert(max(abs(second.log.qTrue(:)-expected.log.qTrue(:)))<1e-10);
    assert(~isequal(second.log.qTrue,b.log.qTrue));
    assert(max(abs(diff(second.log.t)-c2.Ts))<1e-10);
    cf=c; cf.controller='external'; cf.controllerFcn=@unconnected_controller;
    r=run_simulation(line,cf,[],'simulink'); assert(strcmp(r.status,'failed'));
    r=run_simulation(line,c,stopper,'simulink'); assert(strcmp(r.status,'cancelled'));
    names{end+1}='Simulink parity, controlled failure and cancellation';
end
if includeUI
    ui=start_trajectory_designer; app=ui.Controller; app.Fig.Visible='off';
    cleanup=onCleanup(@()delete(ui));
    assert(app.Step==1 && numel(app.UI.controller.Items)==4);
    assert(~isfield(app.UI,'stateNoise') && ~isfield(app.UI,'measurementNoise') && ~isfield(app.UI,'seed'));
    app.goStep(2); assert(app.Step==1);
    assert(~isfield(app.UI,'check') && ~isfield(app.UI,'mpcMode'));
    guiConfig=app.config(); assert(strcmp(guiConfig.algorithm.mode,'nonlinear'));
    app.preparePath(); assert(app.Prepared && ~app.Report.checked); app.confirmPath(); assert(app.Confirmed && app.Step==2);
    assert(abs(app.UI.wrNom.Value-1)<1e-12);
    assert(~isfield(app.UI,'duration') && strcmp(app.UI.desiredDuration.Editable,'off'));
    assert(abs(app.UI.desiredDuration.Value-app.Path.length/app.UI.v0.Value)<1e-12);
    chosenDuration=app.UI.desiredDuration.Value;
    app.UI.controller.Value='sf'; app.controllerChanged();
    assert(strcmp(app.UI.wrNom.Editable,'on'));
    assert(~isfield(app.UI,'autoNominal') && ~isfield(app.UI,'nominalInfo'));
    assert(~isfield(app.UI,'length') && ~isfield(app.UI,'radius') && ~isfield(app.UI,'maxShift'));
    assert(~isfield(app.UI,'model'));
    app.UI.maxOmega.Value=.6;
    cfg=app.config(); assert(isempty(cfg.nominalOverride) && cfg.wMax==.6 && cfg.wMin==-.6);
    assert(abs(rad2deg(cfg.design.wrNom)-1)<1e-12);
    assert(abs(app.UI.pathLength.Value-app.Path.length)<1e-12);
    app.goStep(3); assert(app.Step==3 && app.Confirmed);
    assert(app.config().desiredDuration==chosenDuration);
    app.UI.x0.Value=2; app.invalidate(false); assert(app.Step==2 && app.Confirmed);
    assert(abs(app.UI.wrNom.Value-1)<1e-12);
    assert(~isfield(app.UI,'turnLimit'));
    app.UI.theta0.Value=45; app.updateInitialPose();
    arrow=findobj(app.MainAx,'DisplayName','Initial Heading');
    assert(~isempty(arrow) && abs(atan2(arrow.VData,arrow.UData)-pi/4)<1e-12);
    assert(isempty(findobj(app.MainAx,'DisplayName','Original')));
    app.UI.maxOmega.Value=.01; app.UI.pathType.Value='Circle'; app.invalidate(); app.preparePath();
    assert(app.Prepared && ~app.Report.checked && ~app.Confirmed && app.Step==1 && isempty(app.Result));
    app.confirmPath(); assert(app.Confirmed && app.Step==2);
    app.goStep(3); app.openNoise();
    app.NoiseUI.controlStd.Value=.25; app.NoiseUI.stateNoiseEnabled.Value=true;
    app.applyNoise(); cfg=app.config();
    assert(app.Step==3 && cfg.controlStd==.25 && cfg.stateNoiseEnabled);
    app.openNoise(); app.NoiseUI.controlStd.Value=9; app.closeNoise();
    cfg=app.config(); assert(cfg.controlStd==.25);
    cc=default_config; cc.controller='openloop'; cc.referenceMode='time';
    result=run_simulation(line,cc,[],'matlab');
    app.Result=result; app.Session=[]; app.showResult();
    assert(~isfield(app.UI,'resultValues'));
    assert(all(cellfun(@(h)strcmp(h.HorizontalAlignment,'left'),app.UI.reviewLabels)));
    assert(all(cellfun(@(h)strcmp(h.HorizontalAlignment,'right'),app.UI.reviewValues)));
    assert(strcmp(app.UI.reviewValues{2}.Text,sprintf('%.2f m',app.Path.length)));
    app.finishRun(); assert(isequaln(app.Result,result));
    session=SimulationSession(line,cc,[]); session.observe(0,cc.q0,0); session.control(0,cc.q0);
    app.onFrame(session);
    assert(strcmp(app.LiveTrue.LineStyle,'--'));
    assert(isempty(findobj(app.MainAx,'DisplayName','Estimated Trajectory')));
    names{end+1}='Native three-step UI, nominal override, noise dialog and actual trajectory';
end
summary=struct('passed',{names},'includeSimulink',includeSimulink, ...
    'includeUI',includeUI,'timestamp',char(datetime('now')));
disp(names'); fprintf('PASS: %d test groups\n',numel(names));
end
function [w,m,info]=noTruthController(hat,ref,c,m)
assert(~isfield(c,'q0') && ~isfield(c,'initialEstimateOffset'));
assert(isequal(size(hat),[3 1]) && ~isfield(ref,'qTrue'));
w=0; info=struct('ok',true,'message','Boundary test');
end
function [w,m,info]=badController(~,~,~,m)
w=NaN; info=struct('ok',true);
end
function cancelAtSample(s,index)
if numel(s.Log.t)>=index, s.CancelRequested=true; end
end
