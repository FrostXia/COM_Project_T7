classdef TrajectoryAppController < handle
    % TRAJECTORYAPPCONTROLLER Shared behavior for the three native App Designer views.
    properties
        Root
        MainAx
        EditorAx
        PathFig
        LayoutHost
        MainLayout
        EditorLayout
        NoiseFig = []
        NoiseUI = struct()
        NoiseSettings = struct()
        StepPanels = {}
        Step = 1
        AutoNominal = []
        SpawnStream = []
        Fig
        Ax
        ErrorAxes
        InputAxes
        TimingAxes
        UI = struct()
        Path = []
        Report = []
        Raw = zeros(0,2)
        Prepared = false
        Confirmed = false
        Drawing = false
        Running = false
        StopPending = false
        Session = []
        Result = []
        LiveTrue = []
        LiveRobot = []
        DrawLine = []
        Editable = {}
        DesignerApps = {}
    end
    methods
        function app=TrajectoryAppController(root,visibility,views)
            if nargin<2, visibility='on'; end
            app.Root=root;
            c=SimulationConfig.default_config;
            keys={'controlNoiseEnabled','stateNoiseEnabled','controlStd','stateStd'};
            for k=1:numel(keys), app.NoiseSettings.(keys{k})=c.(keys{k}); end
            assert(nargin>=3,'Trajectory:NativeViews','Start the app with start_app. Native views are required.');
            AppUI.bind_designer_views(app,views,c,visibility);
        end
        function openPathEditor(app,freehand)
            if app.Running, return; end
            app.goStep(1);
            if freehand
                app.beginDraw();
            else
                if strcmp(app.UI.pathType.Value,'Freehand'), app.UI.pathType.Value='Line'; end
                app.invalidate(); app.resetAxes();
                app.status({'Choose Line, Circle or Spiral, preview and confirm.'});
            end
            app.pathControls();
        end
        function editPath(app)
            if app.Running, return; end
            app.goStep(1); app.pathControls();
            if ~isempty(app.Path), app.showPath(); else, app.resetAxes(); end
        end
        function closePathEditor(app)
            % The editor is the main window; closing it exits the application.
            app.closeApp();
        end
        function pathControls(app)
            freehand=strcmp(app.UI.pathType.Value,'Freehand');
            for key={'smoothing','draw'}
                app.UI.(key{1}).Enable=TrajectoryAppController.onoff(freehand);
            end
        end
        function controllerChanged(app)
            app.invalidate(false);
            feedback=ismember(app.UI.controller.Value,{'sf','lqr'});
            app.UI.wrNom.Enable=TrajectoryAppController.onoff(feedback);
            for key={'pole1','pole2','pole3'}
                app.UI.(key{1}).Enable=TrajectoryAppController.onoff(strcmp(app.UI.controller.Value,'sf'));
            end
            for key={'N'}
                app.UI.(key{1}).Enable=TrajectoryAppController.onoff(strcmp(app.UI.controller.Value,'mpc'));
            end
        end
        function updateNominalLabel(app)
            if isempty(app.AutoNominal), app.UI.wrNom.Value=1;
            else, app.UI.wrNom.Value=app.AutoNominal.degreesPerSecond; end
        end
        function goStep(app,step)
            if app.Running, return; end
            if step>1 && ~app.Confirmed
                app.status({'Prepare and confirm the path before continuing.'}); return
            end
            if step==3
                try
                    c=app.config(); SimulationConfig.validate_simulation_config(c);
                    if ismember(c.controller,{'sf','lqr'})
                        ControlDesign.design_feedback_gains(c.design,false,true);
                        if strcmp(c.controller,'sf')
                            assert(numel(unique(c.design.poles))==3,'Trajectory:Poles','State Feedback poles must be distinct.');
                        end
                    end
                    app.updateReview(c);
                catch ME
                    app.status({ME.message}); return
                end
            end
            app.Step=step; app.Drawing=false;
            % Keep just the active native layout in the visible figure.
            % Switching back preserves the path, parameters and current plots.
            if step==1
                app.MainLayout.Parent=app.LayoutHost;
                app.EditorLayout.Parent=app.Fig;
                app.Ax=app.EditorAx;
            else
                app.EditorLayout.Parent=app.LayoutHost;
                app.MainLayout.Parent=app.Fig;
                app.Ax=app.MainAx;
            end
            titles={'Trajectory Setup','Model Selection','Run Simulation'};
            app.UI.stepTitle.Text=sprintf('Step %d: %s',step,titles{step});
            app.Fig.Name=sprintf('Step %d | %s',step,titles{step});
            for j=2:3
                app.StepPanels{j}.Title=sprintf('Step %d: %s',j,titles{j});
                app.StepPanels{j}.Visible=TrajectoryAppController.onoff(j==step);
            end
        end
        function c=config(app)
            c=SimulationConfig.default_config;
            c.v0=app.UI.v0.Value; c.vr=app.UI.vr.Value; c.Ts=app.UI.Ts.Value; c.N=round(app.UI.N.Value);
            if ~isempty(app.Path), c.desiredDuration=app.Path.length/c.vr; end
            c.wMax=app.UI.maxOmega.Value; c.wMin=-c.wMax;
            keys=fieldnames(app.NoiseSettings);
            for k=1:numel(keys), c.(keys{k})=app.NoiseSettings.(keys{k}); end
            c.enforcePathTurnLimit=false;
            c.skipPathValidation=true;
            c.q0=[app.UI.x0.Value;app.UI.y0.Value;deg2rad(app.UI.theta0.Value)];
            c.referenceMode='time'; c.controller=app.UI.controller.Value;
            c.pathSpacing=app.UI.spacing.Value; c.smoothingWindow=round(app.UI.smoothing.Value);
            c.design.v0=c.v0; c.design.vr=c.vr;
            c.design.poles=[app.UI.pole1.Value app.UI.pole2.Value app.UI.pole3.Value];
            c.nominalOverride=[];
            if ~isempty(app.AutoNominal)
                c.design.wrNom=app.AutoNominal.radiansPerSecond;
                c.nominalReference=app.AutoNominal;
                c.nominalReference.source='automatic';
                if ismember(c.controller,{'sf','lqr'}) && app.UI.wrNom.Value~=app.AutoNominal.degreesPerSecond
                    c.nominalOverride=deg2rad(app.UI.wrNom.Value);
                    c.design.wrNom=c.nominalOverride;
                    c.nominalReference.source='manual';
                end
                c.nominalReference.selectedRadiansPerSecond=c.design.wrNom;
            end
            c.algorithm.mode='nonlinear';
        end
        function invalidate(app,pathSetting)
            if nargin<2, pathSetting=true; end
            app.Result=[];
            if pathSetting
                app.Prepared=false; app.Confirmed=false; app.Report=[];
                app.AutoNominal=[]; app.UI.pathLength.Value=0;
                app.UI.desiredDuration.Value=0;
                app.UI.confirm.Enable='off';
                app.updateNominalLabel(); app.pathControls();
                app.goStep(1);
                app.status({'Trajectory settings changed. Preview and confirm again.'});
            elseif app.Step==3
                app.goStep(2);
            end
        end
        function speedChanged(app,referenceChanged)
            if app.Running, return; end
            % A speed edit changes timing/design, not the confirmed path geometry.
            manualNominal=~isempty(app.AutoNominal) && ...
                app.UI.wrNom.Value~=app.AutoNominal.degreesPerSecond;
            app.invalidate(false); app.Session=[];
            if app.Prepared && ~isempty(app.Path)
                c=app.config();
                app.UI.desiredDuration.Value=app.Path.length/c.vr;
                if referenceChanged
                    app.AutoNominal=TrajectoryPath.compute_nominal_turn(app.Path,c.vr,c.nominalTurn);
                    if ~manualNominal, app.updateNominalLabel(); end
                    app.Report=TrajectoryPath.summarize_trajectory(app.Path,c);
                end
            end
            app.status({'Speeds updated for all controllers. Robot speed and desired trajectory speed are independent.'});
        end
        function resetAxes(app)
            cla(app.Ax); hold(app.Ax,'on'); grid(app.Ax,'on'); axis(app.Ax,'equal');
            xlim(app.Ax,[-5 15]); ylim(app.Ax,[-8 12]);
            xlabel(app.Ax,'x (m)'); ylabel(app.Ax,'y (m)');
            title(app.Ax,'Desired Trajectory And Robot Motion');
        end
        function preparePath(app)
            if app.Running, return; end
            app.Prepared=false; app.Confirmed=false; app.Report=[];
            app.UI.confirm.Enable='off';
            try
                c=app.config(); SimulationConfig.validate_simulation_config(c);
                params=c.preset;
                app.Path=TrajectoryPath.generate_trajectory(app.UI.pathType.Value,params,app.Raw,c);
                app.UI.pathLength.Value=app.Path.length;
                app.UI.desiredDuration.Value=app.Path.length/c.vr;
                app.AutoNominal=TrajectoryPath.compute_nominal_turn(app.Path,c.vr,c.nominalTurn);
                app.UI.wrNom.Value=app.AutoNominal.degreesPerSecond;
                app.updateNominalLabel();
                app.Report=TrajectoryPath.summarize_trajectory(app.Path,c);
                app.Prepared=true; app.UI.confirm.Enable='on'; app.showPath();
                lines={'Preview: blue = processed reference; orange stroke appears only for freehand paths.', ...
                    'Inspect the desired trajectory, then confirm it.'};
                app.status(lines);
            catch ME
                app.status({ME.message});
            end
        end
        function showPath(app)
            app.resetAxes(); p=app.Path;
            if isequal(app.Ax,app.EditorAx) && strcmp(p.kind,'freehand')
            plot(app.Ax,p.raw(:,1),p.raw(:,2),':','Color',[1.0 0.65 0.08],'LineWidth',1.4, ...
                'DisplayName','Original','HitTest','off');
            end
            plot(app.Ax,p.xy(:,1),p.xy(:,2),'-','LineWidth',1.6,'Color',[0.1 0.45 0.75], ...
                'DisplayName','Desired Trajectory','HitTest','off');

            c=app.config(); plot(app.Ax,c.q0(1),c.q0(2),'o','Color',[1 .65 .08], ...
                'MarkerFaceColor',[1 .65 .08],'MarkerSize',8,'DisplayName','Robot Initial Position','HitTest','off');
            quiver(app.Ax,c.q0(1),c.q0(2),0.5*cos(c.q0(3)),0.5*sin(c.q0(3)),0, ...
                'Color',[1 .8 .1],'LineWidth',2,'MaxHeadSize',.8, ...
                'DisplayName','Initial Heading','HitTest','off');
            box=[p.xy;c.q0(1:2)']; lo=min(box,[],1)-2; hi=max(box,[],1)+2;
            xlim(app.Ax,[lo(1) hi(1)]); ylim(app.Ax,[lo(2) hi(2)]);
            legend(app.Ax,'show','Location','best');
            app.LiveTrue=[]; app.LiveRobot=[];
        end
        function confirmPath(app)
            if ~app.Prepared || isempty(app.Path), return; end
            app.Confirmed=true;
            app.UI.desiredDuration.Value=app.Path.length/app.UI.vr.Value;
            app.goStep(2); app.randomPose(); app.controllerChanged();
            app.status({'Path confirmed. Step 2: select Open Loop, State Feedback, LQR or MPC.'});
        end
        function beginDraw(app)
            if app.Running, return; end
            app.UI.pathType.Value='Freehand'; app.Raw=zeros(0,2); app.Path=[];
            app.invalidate(); app.resetAxes(); legend(app.Ax,'off');
            disableDefaultInteractivity(app.Ax);
            app.DrawLine=plot(app.Ax,NaN,NaN,'-','Color',[1.0 0.65 0.08],'LineWidth',1.8,'HitTest','off','PickableParts','none');
            app.status({'Press and drag inside the plot to draw one continuous path.', ...
                'Each new drag replaces the previous stroke. Smooth / preview, then Confirm and continue when finished.'});
        end
        function mouseDown(app)
            if app.Running || ~strcmp(app.UI.pathType.Value,'Freehand'), return; end
            if ~strcmp(app.PathFig.SelectionType,'normal'), return; end
            p=app.Ax.CurrentPoint; p=p(1,1:2);
            if ~app.inside(p), return; end
            app.Prepared=false; app.Confirmed=false; app.Report=[]; app.UI.confirm.Enable='off';
            app.AutoNominal=[];
            app.updateNominalLabel(); app.Drawing=true; app.Raw=p;
            if isempty(app.DrawLine) || ~isgraphics(app.DrawLine)
                app.DrawLine=plot(app.Ax,p(1),p(2),'-','Color',[1.0 0.65 0.08],'LineWidth',1.8,'HitTest','off','PickableParts','none');
            end
            app.DrawLine.XData=p(1); app.DrawLine.YData=p(2);
        end
        function mouseMove(app)
            if ~app.Drawing, return; end
            p=app.Ax.CurrentPoint; p=p(1,1:2);
            if app.inside(p) && norm(p-app.Raw(end,:))>0.005
                app.Raw(end+1,:)=p;
                app.DrawLine.XData=app.Raw(:,1); app.DrawLine.YData=app.Raw(:,2);
            end
        end
        function yes=inside(app,p)
            yes=p(1)>=app.Ax.XLim(1) && p(1)<=app.Ax.XLim(2) && p(2)>=app.Ax.YLim(1) && p(2)<=app.Ax.YLim(2);
        end
        function mouseUp(app)
            app.Drawing=false;
        end
        function importPath(app)
            [f,d]=uigetfile('*.csv','Choose a two-column numeric XY path');
            if isequal(f,0), return; end
            try
                raw=readmatrix(fullfile(d,f));
                validateattributes(raw,{'double'},{'ncols',2,'finite','real'});
                app.Raw=raw; app.UI.pathType.Value='Freehand'; app.preparePath();
            catch ME
                app.status({ME.message});
            end
        end
        function updateInitialPose(app)
            if app.Running || isempty(app.Path), return; end
            app.invalidate(false); app.Ax=app.MainAx; app.showPath();
            app.status({'Initial position and heading preview updated.'});
        end
        function randomPose(app)
            if isempty(app.Path), return; end
            if isempty(app.SpawnStream)
                c=SimulationConfig.default_config; app.SpawnStream=RandStream('mt19937ar','Seed',c.seed+10000);
            end
            c=SimulationConfig.default_config; q=TrajectoryPath.sample_initial_pose(app.Path,c,app.SpawnStream);
            app.UI.x0.Value=q(1); app.UI.y0.Value=q(2);
            app.UI.theta0.Value=rad2deg(q(3));
            app.invalidate(false); app.showPath();
            app.status({'Initial pose updated. The confirmed path remains valid.'});
        end
        function runTest(app)
            if app.Running, return; end
            if app.Step~=3 || ~app.Confirmed
                app.status({'Complete Step 1 and Step 2, then review the configuration in Step 3.'}); return
            end
            if isempty(ver('simulink'))
                app.status({'Simulink is required for the Run button. MATLAB verification mode is available only by script.'}); return
            end
            app.Running=true; app.StopPending=false; app.Session=[]; app.Result=[]; app.showPath();
            for i=1:numel(app.Editable), app.Editable{i}.Enable='off'; end
            app.UI.stop.Enable='on';
            clean=onCleanup(@()app.finishRun());
            app.status({'Running in Simulink. Stop takes effect between controller calls.'});
            drawnow;
            try
                app.Result=run_simulation(app.Path,app.config(),@(s)app.onFrame(s),'simulink');
                if isgraphics(app.Fig)
                    app.showResult();
                end
            catch ME
                if isgraphics(app.Fig)
                    details=getReport(ME,'extended','hyperlinks','off');
                    app.status({regexprep(details,'<a[^>]*>(.*?)</a>','$1')});
                end
            end
        end
        function onFrame(app,s)
            app.Session=s;
            if app.StopPending, s.CancelRequested=true; end
            if ~isgraphics(app.Fig), s.CancelRequested=true; return; end
            L=s.Log;
            if isempty(app.LiveTrue) || ~isgraphics(app.LiveTrue)
                app.LiveTrue=plot(app.Ax,NaN,NaN,'--','Color',[.9 .25 .2], ...
                    'LineWidth',1.6,'DisplayName','Robot Trajectory','HitTest','off');
                app.LiveRobot=quiver(app.Ax,0,0,0,0,0,'Color',[1 .8 .1],'LineWidth',2, ...
                    'DisplayName','Robot Heading','HitTest','off');
            end
            app.LiveTrue.XData=L.qTrue(1,:); app.LiveTrue.YData=L.qTrue(2,:);
            q=L.qTrue(:,end);
            set(app.LiveRobot,'XData',q(1),'YData',q(2),'UData',0.5*cos(q(3)),'VData',0.5*sin(q(3)));
            title(app.Ax,sprintf('%s | t = %.1f s',TrajectoryAppController.controllerName(s.Config.controller),L.t(end)));
            drawnow limitrate
        end
        function showResult(app)
            r=app.Result; L=r.log; m=r.metrics; n=m.samples;
            if ~isempty(app.Session), app.onFrame(app.Session); end
            if n>0
                plot(app.ErrorAxes,L.t(1:n),m.positionError,'-',L.t(1:n),m.headingError,'--');
                grid(app.ErrorAxes,'on'); xlabel(app.ErrorAxes,'Time (s)');
                legend(app.ErrorAxes,{'Position Error (m)','Heading Error (rad)'});
                title(app.ErrorAxes,['Errors | ' r.config.referenceMode ' mode']);
                plot(app.InputAxes,L.t(1:n),L.omegaPhysical(1:n)); grid(app.InputAxes,'on');
                xlabel(app.InputAxes,'Time (s)'); ylabel(app.InputAxes,'Actual Angular Velocity (rad/s)');
                plot(app.TimingAxes,L.t(1:n),L.solveTime(1:n)); hold(app.TimingAxes,'on');
                yline(app.TimingAxes,r.config.Ts,'--','Ts','Color',[1 .65 .08]); hold(app.TimingAxes,'off');
                grid(app.TimingAxes,'on'); xlabel(app.TimingAxes,'Time (s)'); ylabel(app.TimingAxes,'Controller Call Time (s)');
            end
            app.status({r.message});
        end
        function updateReview(app,c)
            fields={'Trajectory','Path Length','Desired Trajectory Speed','Robot Speed','Max Omega', ...
                'Controller','Sample Period','Desired Duration', ...
                'Robot Initial X','Robot Initial Y','Robot Initial Heading'};
            values={app.Path.kind,sprintf('%.2f m',app.Path.length), ...
                sprintf('%.3f m/s',c.vr),sprintf('%.3f m/s',c.v0),sprintf('%.3f rad/s',c.wMax), ...
                TrajectoryAppController.controllerName(c.controller),sprintf('%.3f s',c.Ts), ...
                sprintf('%.2f s',app.Path.length/c.vr), ...
                sprintf('%.2f m',c.q0(1)),sprintf('%.2f m',c.q0(2)), ...
                sprintf('%.1f deg',rad2deg(c.q0(3)))};
            if ismember(c.controller,{'sf','lqr'})
                fields{end+1}='Nominal Omega R';
                values{end+1}=sprintf('%.4f deg/s',rad2deg(c.design.wrNom));
            elseif strcmp(c.controller,'mpc')
                fields=[fields,{'MPC Model','MPC Preview Steps'}];
                values=[values,{'Nonlinear',sprintf('%d',c.N)}];
            end
            fields=[fields,{'Control Noise','Measurement Noise','Feedback'}];
            values=[values,{TrajectoryAppController.onoff(c.controlNoiseEnabled), ...
                TrajectoryAppController.onoff(c.stateNoiseEnabled),'Common measured tracking error'}];
            capacity=numel(app.UI.reviewLabels);
            assert(numel(fields)<=capacity,'Trajectory:DesignerReview','The review layout has too few rows.');
            for k=1:capacity
                active=k<=numel(fields);
                app.UI.reviewLabels{k}.Visible=TrajectoryAppController.onoff(active);
                app.UI.reviewValues{k}.Visible=TrajectoryAppController.onoff(active);
                if active
                    app.UI.reviewLabels{k}.Text=fields{k};
                    app.UI.reviewValues{k}.Text=values{k};
                end
            end
        end
        function stopTest(app)
            app.StopPending=true;
            if ~isempty(app.Session), app.Session.CancelRequested=true; end
        end
        function finishRun(app)
            app.Running=false;
            if ~isgraphics(app.Fig), return; end
            for i=1:numel(app.Editable), app.Editable{i}.Enable='on'; end
            app.UI.stop.Enable='off';
            savedResult=app.Result; savedStep=app.Step;
            app.controllerChanged(); app.Result=savedResult; app.goStep(savedStep); app.pathControls();
            app.UI.confirm.Enable=TrajectoryAppController.onoff(app.Prepared && ~isempty(app.Path));
        end
        function openNoise(app)
            if app.Running || app.Step~=3, return; end
            assert(isgraphics(app.NoiseFig),'Trajectory:NativeViews','The native noise view is not available. Restart the app.');
            settings=app.NoiseSettings;
            for key={'controlNoiseEnabled','stateNoiseEnabled'}
                app.NoiseUI.(key{1}).Value=settings.(key{1});
            end
            names={'stateX','stateY','stateTheta'};
            for k=1:numel(names)
                app.NoiseUI.(names{k}).Value=settings.stateStd(k);
            end
            app.NoiseUI.controlStd.Value=settings.controlStd;
            app.noiseControls(); app.NoiseFig.Visible=app.Fig.Visible;
        end
        function noiseControls(app)
            app.NoiseUI.controlStd.Enable=TrajectoryAppController.onoff(app.NoiseUI.controlNoiseEnabled.Value);
            for key={'stateX','stateY','stateTheta'}
                app.NoiseUI.(key{1}).Enable=TrajectoryAppController.onoff(app.NoiseUI.stateNoiseEnabled.Value);
            end
        end
        function applyNoise(app)
            try
                settings=app.NoiseSettings;
                for key={'controlNoiseEnabled','stateNoiseEnabled'}
                    settings.(key{1})=app.NoiseUI.(key{1}).Value;
                end
                settings.controlStd=app.NoiseUI.controlStd.Value;
                settings.stateStd=[app.NoiseUI.stateX.Value;app.NoiseUI.stateY.Value;app.NoiseUI.stateTheta.Value];
                c=app.config(); keys=fieldnames(settings);
                for k=1:numel(keys), c.(keys{k})=settings.(keys{k}); end
                SimulationConfig.validate_simulation_config(c);
                app.NoiseSettings=settings; app.Result=[]; app.Session=[];
                cla(app.ErrorAxes); cla(app.InputAxes); cla(app.TimingAxes);
                app.closeNoise(); app.showPath(); app.goStep(3);
                app.status({'Noise settings applied to the next run.'});
            catch ME
                uialert(app.NoiseFig,ME.message,'Noise Parameters');
            end
        end
        function closeNoise(app)
            if isgraphics(app.NoiseFig), app.NoiseFig.Visible='off'; end
        end
        function exportResult(app)
            if isempty(app.Result), app.status({'Run a test before exporting.'}); return; end
            try
                folder=ExperimentResults.export_experiment(app.Result,app.Root);
                app.status({['Saved desired path, evaluation, robot trajectory, errors, angular velocity and timing CSV files: ' folder]});
            catch ME
                app.status({ME.message});
            end
        end
        function closeApp(app)
            app.stopTest();
            app.Drawing=false;
            % The main figure's DeleteFcn releases the inactive layout and noise app.
            if isgraphics(app.Fig), delete(app.Fig); end
        end
        function status(app,lines)
            if isgraphics(app.Fig), app.UI.status.Value=lines(:); end
            if isfield(app.UI,'pathStatus') && isgraphics(app.UI.pathStatus)
                app.UI.pathStatus.Value=lines(:);
            end
        end
    end
    methods (Static)
        function value=controllerName(key)
            switch key
                case 'sf', value='State Feedback';
                case 'lqr', value='LQR';
                case 'mpc', value='MPC';
                case 'openloop', value='Open Loop';
                otherwise, value=key;
            end
        end
        function value=onoff(flag)
            if flag, value='on'; else, value='off'; end
        end
    end
end
