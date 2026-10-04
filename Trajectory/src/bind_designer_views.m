function bind_designer_views(app,views,c,visibility)
% BIND_DESIGNER_VIEWS Connect saved App Designer components to shared behavior.
app.DesignerApps={views.Main,views.Editor,views.Noise};
app.Fig=views.Main.UIFigure; app.PathFig=views.Editor.UIFigure;
app.NoiseFig=views.Noise.UIFigure;
figures=[app.Fig,app.PathFig,app.NoiseFig];
for f=figures, f.Visible='off'; end
for f=figures
    components=findall(f,'-regexp','Tag','^view_ui_');
    for k=1:numel(components)
        key=erase(components(k).Tag,'view_ui_'); app.UI.(key)=components(k);
    end
end
for key={'MainAx','EditorAx','ErrorAxes','InputAxes','TimingAxes'}
    app.(key{1})=tagged(figures,['view_' key{1}]);
end
app.Ax=app.MainAx;
for k=1:3, app.StepPanels{k}=tagged(figures,sprintf('view_Step%d',k)); end
app.UI.reviewLabels={}; app.UI.reviewValues={};
for k=1:18
    app.UI.reviewLabels{k}=tagged(figures,sprintf('view_ReviewLabel%d',k));
    app.UI.reviewValues{k}=tagged(figures,sprintf('view_ReviewValue%d',k));
end
buttons={ ...
    'preset',@()app.openPathEditor(false); 'handdraw',@()app.openPathEditor(true); ...
    'editPath',@()app.editPath(); 'next1',@()app.goStep(2); ...
    'random',@()app.randomPose(); 'updatePose',@()app.updateInitialPose(); ...
    'back2',@()app.goStep(1); 'next2',@()app.goStep(3); ...
    'noise',@()app.openNoise(); 'run',@()app.runTest(); 'stop',@()app.stopTest(); ...
    'export',@()app.exportResult(); 'back3',@()app.goStep(2); ...
    'draw',@()app.beginDraw(); 'import',@()app.importPath(); ...
    'preview',@()app.preparePath(); 'confirm',@()app.confirmPath(); ...
    'closeEditor',@()app.closePathEditor()};
for k=1:size(buttons,1)
    key=buttons{k,1}; action=buttons{k,2};
    app.UI.(key).ButtonPushedFcn=@(~,~)action();
    if ~strcmp(key,'stop'), app.Editable{end+1}=app.UI.(key); end
end
for key={'pathType','smoothing','spacing','v0'}
    app.UI.(key{1}).ValueChangedFcn=@(~,~)app.invalidate(true);
    app.Editable{end+1}=app.UI.(key{1});
end
for key={'wrNom','pole1','pole2','pole3','N','Ts','maxOmega','x0','y0','theta0'}
    app.UI.(key{1}).ValueChangedFcn=@(~,~)app.invalidate(false);
    app.Editable{end+1}=app.UI.(key{1});
end
app.UI.controller.ItemsData={'openloop','sf','lqr','mpc'};
app.UI.controller.Value='openloop';
app.UI.controller.ValueChangedFcn=@(~,~)app.controllerChanged();
app.Editable{end+1}=app.UI.controller;
% Experiment defaults continue to come from the code configuration.
defaults=struct('wrNom',1,'pole1',c.design.poles(1),'pole2',c.design.poles(2), ...
    'pole3',c.design.poles(3),'N',c.N,'Ts',c.Ts,'desiredDuration',0, ...
    'maxOmega',c.wMax,'x0',c.q0(1),'y0',c.q0(2),'theta0',rad2deg(c.q0(3)), ...
    'smoothing',c.smoothingWindow,'spacing',c.pathSpacing,'v0',c.v0,'pathLength',0);
keys=fieldnames(defaults);
for k=1:numel(keys), app.UI.(keys{k}).Value=defaults.(keys{k}); end
app.UI.pathType.Value='Line'; app.UI.wrNom.Editable='on'; app.UI.pathLength.Editable='off';
app.UI.desiredDuration.Editable='off';
app.UI.confirm.Enable='off'; app.UI.next1.Enable='off'; app.UI.stop.Enable='off';
for key={'measurementNoiseEnabled','controlNoiseEnabled','stateNoiseEnabled', ...
        'measurementX','measurementY','measurementTheta','controlStd','stateX','stateY','stateTheta'}
    app.NoiseUI.(key{1})=tagged(figures,['view_noise_' key{1}]);
end
for key={'measurementNoiseEnabled','controlNoiseEnabled','stateNoiseEnabled'}
    app.NoiseUI.(key{1}).ValueChangedFcn=@(~,~)app.noiseControls();
end
apply=tagged(figures,'view_noise_apply'); apply.ButtonPushedFcn=@(~,~)app.applyNoise();
cancel=tagged(figures,'view_noise_cancel'); cancel.ButtonPushedFcn=@(~,~)app.closeNoise();
note=tagged(figures,'view_noise_note');
if strcmp(c.sensorMode,'position')
    note.Text='The current sensor measures X and Y; heading is estimated.';
else
    note.Text='Measurement noise affects X, Y and heading.';
end
app.Fig.CloseRequestFcn=@(~,~)app.closeApp();
app.Fig.DeleteFcn=@(~,~)deleteAuxiliary(views);
app.PathFig.CloseRequestFcn=@(~,~)app.closePathEditor();
app.NoiseFig.CloseRequestFcn=@(~,~)app.closeNoise();
app.PathFig.WindowStyle='modal'; app.NoiseFig.WindowStyle='modal';
app.EditorAx.ButtonDownFcn=@(~,~)app.mouseDown();
app.PathFig.WindowButtonMotionFcn=@(~,~)app.mouseMove();
app.PathFig.WindowButtonUpFcn=@(~,~)app.mouseUp();
for ax=[app.MainAx app.EditorAx app.ErrorAxes app.InputAxes app.TimingAxes]
    ax.ColorOrder=trajectory_plot_colors;
end

app.resetAxes(); app.controllerChanged(); app.goStep(1);
app.status({'Step 1: load a preset or draw a path in the path editor.'});
app.Fig.Visible=visibility;
end

function h=tagged(figures,tag)
h=findall(figures,'Tag',tag);
assert(isscalar(h),'Trajectory:DesignerTag','Expected one component with Tag %s.',tag);
end

function deleteAuxiliary(views)
% Also release child views when the App Designer object is deleted directly.
for key={'Editor','Noise'}
    view=views.(key{1});
    if isvalid(view), delete(view); end
end
end
