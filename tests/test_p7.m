function summary = test_p7(includeSimulink,includeUI)
% TEST_P7 Verify geometry, estimation, reproducibility and the model harness.
if nargin<1, includeSimulink=true; end
if nargin<2, includeUI=false; end
root=fileparts(fileparts(mfilename('fullpath')));
addpath(root); setup_project7;
names={};
c=p7_defaults;
q=[1;2;0.4]; [actual,F]=p7_motion(q,0,c);
assert(norm(actual-q-[c.v0*c.Ts*cos(q(3));c.v0*c.Ts*sin(q(3));0])<1e-12);
delta=1e-6; qp=q; qp(3)=qp(3)+delta; qm=q; qm(3)=qm(3)-delta;
assert(norm((p7_motion(qp,0,c)-p7_motion(qm,0,c))/(2*delta)-F(:,3))<1e-8);
cTurn=c; cTurn.Ts=2*pi;
assert(norm(p7_motion([0;0;0],1,cTurn)-[0;0;2*pi])<1e-12);
names{end+1}='Exact plant integration and Jacobian';

line=p7_make_path('line',[12 3 .15 1],[],c); report=p7_check_path(line,c);
assert(report.ok && max(abs(line.kappa))==0);
circle=p7_make_path('circle',[12 3 .15 1],[],c);
assert(max(abs(circle.kappa-1/3))<1e-12);
small=p7_make_path('circle',[12 .1 .15 1],[],c);
report=p7_check_path(small,c); assert(~report.ok);
spiral=p7_make_path('spiral',[12 3 .15 1],[],c);
assert(all(diff(spiral.s)>0) && all(isfinite(spiral.theta)));
raw=[linspace(0,8,100)',0.2*sin(linspace(0,3,100)')];
drawn=p7_make_path('freehand',[],raw,c);
assert(all(diff(drawn.s)>0));
names{end+1}='Preset and freehand geometry; impossible radius rejected';
test_p7_smoothing;
names{end+1}='Bounded adaptive smoothing: jitter removal and real bend rejection';

[qtail,wtail]=p7_path_query(line,[line.length,line.length+1],c);
assert(abs(qtail(1,2)-qtail(1,1)-1)<1e-10 && all(wtail==0));
ct=c; ct.referenceMode='time';
[r,~]=p7_reference([100;100;0],1,line,ct,[]);
assert(isequal(size(r.q),[3,c.N+1]) && isequal(size(r.w),[c.N,1]));
assert(abs(r.q(1,1)-c.v0)<1e-9);
names{end+1}='Reference windows and non-frozen endpoint preview';

ce=c; ce.qHat0=[0;0;pi-0.01]; ce.sensorMode='pose'; ce.P0=eye(3);
[hat,m]=p7_estimator([0;0;-pi+0.01],0,ce,[]);
assert(abs(hat(3)-pi)<0.02 && min(eig(m.P))>=-1e-10);
cd=ce; cd.estimatorMode='deadreckoning';
[hat1,mem]=p7_estimator([999;999;0],0,cd,[]);
[hat2,~]=p7_estimator([-999;-999;0],0,cd,mem);
assert(norm(hat1-cd.qHat0)<1e-12 && norm(hat2-p7_motion(hat1,0,cd))<1e-12);
names{end+1}='Angular innovation and prediction-only state estimation';

c.duration=2; c.stateNoiseEnabled=true;
a=p7_run(line,c,[],'matlab'); b=p7_run(line,c,[],'matlab');
assert(isequal(a.log.qTrue,b.log.qTrue) && isequaln(a.log.measurement,b.log.measurement));
assert(all(isfinite(a.log.qHat(:))) && all(a.log.omega<=c.wMax+eps));
assert(norm(a.log.qTrue-a.log.qHat,'fro')>0);
names{end+1}='Seeded noisy runs and distinct estimated state';

cf=c; cf.controller='external'; cf.controllerFcn=@noTruthController;
r=p7_run(line,cf,[],'matlab'); assert(strcmp(r.status,'completed'));
cf.controllerFcn=@p7_unconnected_controller;
r=p7_run(line,cf,[],'matlab'); assert(strcmp(r.status,'failed') && r.metrics.failureCount==1);
cf.controllerFcn=@badController;
r=p7_run(line,cf,[],'matlab'); assert(strcmp(r.status,'failed'));
names{end+1}='Algorithm boundary and failed controller handling';

stopper=@(s) cancelAtSample(s,3);
r=p7_run(line,c,stopper,'matlab'); assert(strcmp(r.status,'cancelled'));
names{end+1}='Cancellation is distinct from completed tracking';

if includeSimulink
    a=p7_run(line,c,[],'matlab'); b=p7_run(line,c,[],'simulink');
    assert(~strcmp(b.status,'failed'),b.message);
    assert(isequal(size(a.log.qTrue),size(b.log.qTrue)));
    assert(max(abs(a.log.qTrue(:)-b.log.qTrue(:)))<1e-10);
    assert(max(abs(a.log.qHat(:)-b.log.qHat(:)))<1e-10);
    cf=c; cf.controller='external'; cf.controllerFcn=@p7_unconnected_controller;
    r=p7_run(line,cf,[],'simulink'); assert(strcmp(r.status,'failed'));
    r=p7_run(line,c,stopper,'simulink'); assert(strcmp(r.status,'cancelled'));
    names{end+1}='Simulink parity, controlled failure and cancellation';
end
if includeUI
    app=P7App(fullfile(root,'Trajectory'),'off'); cleanup=onCleanup(@()delete(app.Fig));
    app.checkPath(); assert(app.Report.ok); app.confirmPath(); assert(app.Confirmed);
    app.UI.turnLimit.Value=.01; app.UI.pathType.Value='Circle'; app.invalidate(); app.checkPath();
    assert(~app.Report.ok && ~app.Confirmed);
    names{end+1}='UI creation, confirmation and invalid-path gating';
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
