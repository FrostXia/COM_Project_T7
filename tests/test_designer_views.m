function test_designer_views(includeSimulink)
% TEST_DESIGNER_VIEWS Verify native views, callbacks and optional Simulink execution.
if nargin<1, includeSimulink=false; end
root=fileparts(fileparts(mfilename('fullpath'))); addpath(root); setup_paths;
ui=start_trajectory_designer;
cleanup=onCleanup(@()deleteView(ui));
app=ui.Controller; app.Fig.Visible='off';
assert(isa(app,'TrajectoryAppController') && numel(app.DesignerApps)==3);
assert(strcmp(app.UI.wrNom.Editable,'on'));
assert(strcmp(app.UI.pathLength.Editable,'off'));
assert(~isfield(app.UI,'duration') && strcmp(app.UI.desiredDuration.Editable,'off'));
for kind={'Line','Circle','Spiral'}
    app.UI.pathType.Value=kind{1}; app.preparePath();
    assert(app.Prepared && app.Path.length>0);
end
app.UI.pathType.Value='Freehand';
x=linspace(0,6,120)'; app.Raw=[x,sin(x)]; app.preparePath();
assert(app.Prepared && strcmp(app.Path.kind,'freehand'));
callback=app.UI.confirm.ButtonPushedFcn; callback(app.UI.confirm,[]);
assert(app.Step==2 && app.Confirmed);
for key={'openloop','sf','lqr','mpc'}
    app.UI.controller.Value=key{1};
    callback=app.UI.controller.ValueChangedFcn; callback(app.UI.controller,[]);
    app.goStep(3);
    assert(app.Step==3,'Trajectory:Review','Review failed for %s',key{1});
end
app.UI.controller.Value='lqr'; app.controllerChanged();
app.UI.wrNom.Value=-7;
callback=app.UI.wrNom.ValueChangedFcn; callback(app.UI.wrNom,[]);
cfg=app.config();
assert(abs(cfg.nominalOverride-deg2rad(-7))<1e-12);
assert(strcmp(cfg.nominalReference.source,'manual'));
resolved=prepare_controller_design(app.Path,cfg);
assert(abs(resolved.design.wrNom-cfg.nominalOverride)<1e-12);
app.preparePath(); app.confirmPath();
assert(isempty(app.config().nominalOverride));
app.goStep(3);
callback=app.UI.noise.ButtonPushedFcn; callback(app.UI.noise,[]);
app.NoiseUI.controlStd.Value=0.15;
h=findall(app.NoiseFig,'Tag','view_noise_apply'); callback=h.ButtonPushedFcn; callback(h,[]);
assert(app.NoiseSettings.controlStd==0.15 && app.Step==3);
if includeSimulink
    app.UI.pathType.Value='Freehand'; app.Raw=[linspace(0,.25,20)',zeros(20,1)];
    app.preparePath(); app.confirmPath();
    assert(app.Prepared && app.Confirmed);
    app.UI.controller.Value='openloop'; app.controllerChanged();
    assert(abs(app.UI.desiredDuration.Value-.5)<1e-12);
    app.UI.x0.Value=0; app.UI.y0.Value=0; app.UI.theta0.Value=0;
    app.goStep(3);
    callback=app.UI.run.ButtonPushedFcn; callback(app.UI.run,[]);
    assert(~isempty(app.Result),'Trajectory:Run','Run callback did not return a result.');
    assert(~strcmp(app.Result.status,'failed'),'Trajectory:Run','%s',app.Result.message);
    assert(~app.Running && strcmp(app.UI.stop.Enable,'off'));
    assert(numel(app.Result.log.t)>1);
    assert(abs(app.Result.config.desiredDuration-.5)<1e-12);
    assert(app.Result.log.t(end)<=.5+1e-12);
end
children=[app.PathFig,app.NoiseFig];
delete(ui);
assert(~any(isgraphics(children)),'Trajectory:ChildCleanup','Child windows remained open.');
disp('PASS: native App Designer views and controller integration');
end

function deleteView(ui)
if isvalid(ui), delete(ui); end
end
