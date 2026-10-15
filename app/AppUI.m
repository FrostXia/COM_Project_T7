classdef AppUI
    % APPUI Related operations grouped as static methods.
    methods (Static)

        function bind_designer_views(app,views,c,visibility)
        % BIND_DESIGNER_VIEWS Connect saved App Designer components to shared behavior.
        app.DesignerApps={views.Main,views.Editor,views.Noise};
        app.Fig=views.Main.UIFigure; app.PathFig=app.Fig;
        app.LayoutHost=views.Editor.UIFigure; app.NoiseFig=views.Noise.UIFigure;
        figures=[app.Fig,app.LayoutHost,app.NoiseFig];
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
        % Reuse the native editor as Step 1 in the application's only main window.
        % The hidden editor figure holds whichever saved layout is currently inactive.
        app.MainLayout=app.UI.stepTitle.Parent;
        app.EditorLayout=app.EditorAx.Parent;
        app.Fig.Position(3:4)=app.LayoutHost.Position(3:4);
        % Remove unused components in the saved main layout; the editor supplies Step 1.
        unusedMainStep=findall(app.Fig,'Tag','view_Step1');
        delete(unusedMainStep);
        unusedMainKeys={'preset','handdraw','editPath','next1','pathSummary'};
        for k=1:numel(unusedMainKeys)
            if isfield(app.UI,unusedMainKeys{k}), app.UI=rmfield(app.UI,unusedMainKeys{k}); end
        end
        app.StepPanels={[],[],[]};
        for k=2:3, app.StepPanels{k}=tagged(figures,sprintf('view_Step%d',k)); end
        app.UI.reviewLabels={}; app.UI.reviewValues={};
        for k=1:18
            app.UI.reviewLabels{k}=tagged(figures,sprintf('view_ReviewLabel%d',k));
            app.UI.reviewValues{k}=tagged(figures,sprintf('view_ReviewValue%d',k));
        end
        AppUI.configure_speed_controls(app,c);
        buttons={ ...
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
        for key={'pathType','smoothing','spacing'}
            app.UI.(key{1}).ValueChangedFcn=@(~,~)app.invalidate(true);
            app.Editable{end+1}=app.UI.(key{1});
        end
        app.UI.vr.ValueChangedFcn=@(~,~)app.speedChanged(true);
        app.UI.v0.ValueChangedFcn=@(~,~)app.speedChanged(false);
        app.Editable=[app.Editable,{app.UI.vr,app.UI.v0}];
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
            'smoothing',c.smoothingWindow,'spacing',c.pathSpacing,'v0',c.v0,'vr',SimulationConfig.reference_speed(c),'pathLength',0);
        keys=fieldnames(defaults);
        for k=1:numel(keys), app.UI.(keys{k}).Value=defaults.(keys{k}); end
        app.UI.pathType.Value='Freehand'; app.UI.wrNom.Editable='on'; app.UI.pathLength.Editable='off';
        app.UI.desiredDuration.Editable='off';
        app.UI.confirm.Enable='off'; app.UI.stop.Enable='off';
        for key={'controlNoiseEnabled','stateNoiseEnabled','controlStd','stateX','stateY','stateTheta'}
            app.NoiseUI.(key{1})=tagged(figures,['view_noise_' key{1}]);
        end
        for key={'controlNoiseEnabled','stateNoiseEnabled'}
            app.NoiseUI.(key{1}).ValueChangedFcn=@(~,~)app.noiseControls();
        end
        apply=tagged(figures,'view_noise_apply'); apply.ButtonPushedFcn=@(~,~)app.applyNoise();
        cancel=tagged(figures,'view_noise_cancel'); cancel.ButtonPushedFcn=@(~,~)app.closeNoise();
        note=tagged(figures,'view_noise_note');
        note.Text='Max = 3 sigma. Truncated Gaussian; outliers are redrawn.';
        app.NoiseUI.stateNoiseEnabled.Text='Measurement Noise';
        labels=findall(app.NoiseFig,'-isa','matlab.ui.control.Label');
        oldLabels={'Control Omega (rad/s)','State X Error (m)','State Y Error (m)','State Orientation Error (rad)'};
        newLabels={'Control Omega Max (rad/s)','X Error Max (m)','Y Error Max (m)','Orientation Max (rad)'};
        for k=1:numel(labels)
            index=find(strcmp(labels(k).Text,oldLabels),1);
            if ~isempty(index), labels(k).Text=newLabels{index}; end
        end
        app.Fig.CloseRequestFcn=@(~,~)app.closeApp();
        app.Fig.DeleteFcn=@(~,~)deleteAuxiliary(views);
        app.NoiseFig.CloseRequestFcn=@(~,~)app.closeNoise();
        app.Fig.WindowStyle='normal'; app.NoiseFig.WindowStyle='modal';
        app.EditorAx.ButtonDownFcn=@(~,~)app.mouseDown();
        app.PathFig.WindowButtonMotionFcn=@(~,~)app.mouseMove();
        app.PathFig.WindowButtonUpFcn=@(~,~)app.mouseUp();
        for ax=[app.MainAx app.EditorAx app.ErrorAxes app.InputAxes app.TimingAxes]
            ax.ColorOrder=AppUI.trajectory_plot_colors;
        end

        app.resetAxes(); app.controllerChanged(); app.goStep(1);
        app.beginDraw();
        app.Fig.Visible=visibility;
        end

        function configure_speed_controls(app,c)
        % CONFIGURE_SPEED_CONTROLS Place independent reference/robot speeds in Step 2.
        % Reuse the saved reference-speed field and add only the robot-speed row.
        app.UI.vr=app.UI.v0;
        app.UI.vr.Tag='view_ui_vr';
        source=app.UI.vr.Parent; sourceRow=app.UI.vr.Layout.Row;
        sourceHeights=source.RowHeight;
        label=[]; children=source.Children;
        for k=1:numel(children)
            h=children(k);
            if isa(h,'matlab.ui.control.Label') && isequal(h.Layout.Row,sourceRow) && isequal(h.Layout.Column,1)
                label=h; break
            end
        end
        assert(~isempty(label),'Trajectory:SpeedLayout','Reference speed label was not found.');
        target=app.UI.maxOmega.Parent; row=app.UI.maxOmega.Layout.Row;
        heights=target.RowHeight;
        targetHeights=[heights(1:row-1),heights(row),heights(row),heights(row:end)];
        target.RowHeight=targetHeights;
        children=target.Children;
        for k=1:numel(children)
            rows=children(k).Layout.Row;
            rows(rows>=row)=rows(rows>=row)+2;
            children(k).Layout.Row=rows;
        end
        label.Parent=target; label.Layout.Row=row; label.Layout.Column=1;
        label.Tag='view_label_referenceSpeed';
        app.UI.vr.Parent=target; app.UI.vr.Layout.Row=row; app.UI.vr.Layout.Column=2;
        robotLabel=uilabel(target,'Text','Robot speed (m/s)','WordWrap','on', ...
            'Tag','view_label_robotSpeed');
        robotLabel.Layout.Row=row+1; robotLabel.Layout.Column=1;
        app.UI.v0=uieditfield(target,'numeric','Limits',app.UI.vr.Limits, ...
            'Value',c.v0,'Tag','view_ui_v0');
        app.UI.v0.Layout.Row=row+1; app.UI.v0.Layout.Column=2;
        target.RowHeight=targetHeights;
        % Remove the vacated speed row from the editor without changing its controls.
        children=source.Children;
        for k=1:numel(children)
            rows=children(k).Layout.Row;
            rows(rows>sourceRow)=rows(rows>sourceRow)-1;
            children(k).Layout.Row=rows;
        end
        sourceHeights(sourceRow)=[];
        source.RowHeight=sourceHeights;
        end

        function colors = trajectory_plot_colors
        % TRAJECTORY_PLOT_COLORS Chromatic data colors for light and dark plot themes.
        colors=[.2 .6 .95; 1 .65 .08; .85 .25 .3; .15 .7 .5; .65 .4 .9; .1 .75 .8; .9 .4 .7];
        end
    end
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
