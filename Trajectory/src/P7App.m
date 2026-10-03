classdef P7App < handle
    % P7APP Programmatic MATLAB app for path preparation and Simulink tests.
    properties
        Root
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
        LiveHat = []
        LiveRobot = []
        DrawLine = []
        Editable = {}
    end
    methods
        function app=P7App(root,visibility)
            if nargin<2, visibility='on'; end
            app.Root=root;
            app.Fig=uifigure('Name','Project 7 | Trajectory','Position',[80 60 1280 820], ...
                'Visible',visibility,'CloseRequestFcn',@(~,~)app.closeApp());
            main=uigridlayout(app.Fig,[2 2]);
            main.ColumnWidth={335,'1x'}; main.RowHeight={'1x',130};
            panel=uipanel(main,'Title','Experiment settings','Scrollable','on');
            panel.Layout.Row=[1 2]; panel.Layout.Column=1;
            grid=uigridlayout(panel,[41 2]);
            grid.Scrollable='on';
            grid.ColumnWidth={155,'1x'}; grid.RowHeight=repmat({28},1,41);
            c=p7_defaults;
            app.addDrop(grid,1,'Path source','pathType',{'Line','Circle','Spiral','Freehand'},'Line');
            app.addNumber(grid,2,'Line length (m)','length',12,[0.1 1000]);
            app.addNumber(grid,3,'Radius / start radius','radius',3,[0.01 100]);
            app.addNumber(grid,4,'Spiral growth (m/rad)','growth',0.15,[0 10]);
            app.addNumber(grid,5,'Turns','turns',1,[0.1 20]);
            app.addNumber(grid,6,'Initial smoothing samples','smoothing',11,[1 101]);
            app.addNumber(grid,7,'Path spacing (m)','spacing',c.pathSpacing,[0.005 1]);
            app.addNumber(grid,8,'Max smoothing shift (m)','maxShift',c.maxPathDeviation,[0.001 5]);
            app.addButton(grid,9,'Draw / clear','draw',@()app.beginDraw());
            app.addButton(grid,10,'Import XY CSV','import',@()app.importPath());
            app.addButton(grid,11,'Smooth / preview','preview',@()app.preparePath());
            app.addButton(grid,12,'Validate path','check',@()app.checkPath());
            app.addButton(grid,13,'Confirm processed path','confirm',@()app.confirmPath());
            app.UI.confirm.Enable='off';
            app.addDrop(grid,14,'Reference mode','reference',{'path','time'},'path');
            app.addDrop(grid,15,'Controller','controller',{'demo','openloop','sf','lqr','mpc','external'},'demo');
            app.addNumber(grid,16,'Fixed speed (m/s)','v0',c.v0,[0.01 10]);
            app.addNumber(grid,17,'Turn limit (rad/s)','turnLimit',c.wMax,[0.01 20]);
            app.addNumber(grid,18,'Sample period (s)','Ts',c.Ts,[0.01 1]);
            app.addNumber(grid,19,'Preview steps','N',c.N,[1 1000]);
            app.addNumber(grid,20,'Max duration (s)','duration',c.duration,[0.1 600]);
            app.addNumber(grid,21,'Initial x (m)','x0',c.q0(1),[-1000 1000]);
            app.addNumber(grid,22,'Initial y (m)','y0',c.q0(2),[-1000 1000]);
            app.addNumber(grid,23,'Initial heading (deg)','theta0',rad2deg(c.q0(3)),[-3600 3600]);
            app.addButton(grid,24,'Random initial pose','random',@()app.randomPose());
            app.addDrop(grid,25,'State source','estimator',{'ekf','deadreckoning','ideal'},'ekf');
            app.addDrop(grid,26,'Measured quantities','sensor',{'position','pose'},'position');
            app.addNumber(grid,27,'Prior x offset (m)','dxHat',c.initialEstimateOffset(1),[-100 100]);
            app.addNumber(grid,28,'Prior y offset (m)','dyHat',c.initialEstimateOffset(2),[-100 100]);
            app.addNumber(grid,29,'Prior angle offset (deg)','dthetaHat',rad2deg(c.initialEstimateOffset(3)),[-180 180]);
            app.addCheck(grid,30,'State noise','stateNoise',false);
            app.addCheck(grid,31,'Measurement noise','measurementNoise',true);
            app.addNumber(grid,32,'Random seed','seed',c.seed,[0 2^32-3]);
            app.addButton(grid,33,'Run in Simulink','run',@()app.runTest());
            app.addButton(grid,34,'Stop test','stop',@()app.stopTest());
            app.UI.stop.Enable='off';
            app.addButton(grid,35,'Open Simulink model','model',@()app.openModel());
            app.addButton(grid,36,'Export last result','export',@()app.exportResult());
            app.addButton(grid,37,'Open Chinese guide','guide',@()edit(fullfile(fileparts(app.Root),'docs','使用说明.md')));
            app.addDrop(grid,40,'MPC prediction','mpcMode',{'nonlinear','ltv'},'nonlinear');
            note=uilabel(grid,'Text','demo is a test controller, NOT MPC.','WordWrap','on');
            note.Layout.Row=[38 39]; note.Layout.Column=[1 2];
            tabs=uitabgroup(main); tabs.Layout.Row=1; tabs.Layout.Column=2;
            t=uitab(tabs,'Title','Path and motion'); g=uigridlayout(t,[1 1]);
            app.Ax=uiaxes(g); app.Ax.ButtonDownFcn=@(~,~)app.mouseDown();
            t=uitab(tabs,'Title','Tracking errors'); g=uigridlayout(t,[1 1]); app.ErrorAxes=uiaxes(g);
            t=uitab(tabs,'Title','Control and timing'); g=uigridlayout(t,[2 1]);
            app.InputAxes=uiaxes(g); app.TimingAxes=uiaxes(g);
            app.UI.status=uitextarea(main,'Editable','off');
            app.UI.status.Layout.Row=2; app.UI.status.Layout.Column=2;
            app.Fig.WindowButtonMotionFcn=@(~,~)app.mouseMove();
            app.Fig.WindowButtonUpFcn=@(~,~)app.mouseUp();
            app.resetAxes();
            app.status({'Choose a preset or draw a path, then Smooth / preview, then Validate path.', ...
                'Inspect the processed path and click Confirm processed path.', ...
                'Default feedback: EKF with noisy position measurements.', ...
                'State noise std per sample: [0.003 m, 0.003 m, 0.005 rad]. Measurement std: [0.03 m, 0.03 m, 0.03 rad].'});
        end
        function addNumber(app,g,row,label,key,value,limits)
            l=uilabel(g,'Text',label); l.Layout.Row=row; l.Layout.Column=1;
            h=uieditfield(g,'numeric','Value',value,'Limits',limits,'ValueChangedFcn',@(~,~)app.invalidate());
            h.Layout.Row=row; h.Layout.Column=2; app.UI.(key)=h; app.Editable{end+1}=h;
        end
        function addDrop(app,g,row,label,key,items,value)
            l=uilabel(g,'Text',label); l.Layout.Row=row; l.Layout.Column=1;
            h=uidropdown(g,'Items',items,'Value',value,'ValueChangedFcn',@(~,~)app.invalidate());
            h.Layout.Row=row; h.Layout.Column=2; app.UI.(key)=h; app.Editable{end+1}=h;
        end
        function addCheck(app,g,row,label,key,value)
            h=uicheckbox(g,'Text',label,'Value',value,'ValueChangedFcn',@(~,~)app.invalidate());
            h.Layout.Row=row; h.Layout.Column=[1 2]; app.UI.(key)=h; app.Editable{end+1}=h;
        end
        function addButton(app,g,row,label,key,callback)
            h=uibutton(g,'Text',label,'ButtonPushedFcn',@(~,~)callback());
            h.Layout.Row=row; h.Layout.Column=[1 2]; app.UI.(key)=h;
            if ~ismember(key,{'stop','guide'}), app.Editable{end+1}=h; end
        end
        function c=config(app)
            c=p7_defaults;
            c.algorithm=struct('mode',app.UI.mpcMode.Value);
            c.v0=app.UI.v0.Value; c.Ts=app.UI.Ts.Value; c.N=round(app.UI.N.Value);
            c.duration=app.UI.duration.Value; c.wMax=app.UI.turnLimit.Value; c.wMin=-c.wMax;
            c.q0=[app.UI.x0.Value;app.UI.y0.Value;deg2rad(app.UI.theta0.Value)];
            c.initialEstimateOffset=[app.UI.dxHat.Value;app.UI.dyHat.Value;deg2rad(app.UI.dthetaHat.Value)];
            c.referenceMode=app.UI.reference.Value; c.controller=app.UI.controller.Value;
            c.estimatorMode=app.UI.estimator.Value; c.sensorMode=app.UI.sensor.Value;
            c.stateNoiseEnabled=app.UI.stateNoise.Value; c.measurementNoiseEnabled=app.UI.measurementNoise.Value;
            c.seed=round(app.UI.seed.Value); c.pathSpacing=app.UI.spacing.Value;
            c.smoothingWindow=round(app.UI.smoothing.Value);
            c.maxPathDeviation=app.UI.maxShift.Value;
        end
        function invalidate(app)
            app.Prepared=false; app.Confirmed=false; app.Report=[]; app.UI.confirm.Enable='off';
            app.status({'Settings changed. Smooth / preview and validate the path and confirm it again.'});
        end
        function resetAxes(app)
            cla(app.Ax); hold(app.Ax,'on'); grid(app.Ax,'on'); axis(app.Ax,'equal');
            xlim(app.Ax,[-5 15]); ylim(app.Ax,[-8 12]);
            xlabel(app.Ax,'x (m)'); ylabel(app.Ax,'y (m)');
            title(app.Ax,'Reference path and robot motion');
        end
        function preparePath(app)
            if app.Running, return; end
            app.Prepared=false; app.Confirmed=false; app.Report=[];
            app.UI.confirm.Enable='off';
            try
                c=app.config(); p7_validate_config(c);
                params=[app.UI.length.Value app.UI.radius.Value app.UI.growth.Value app.UI.turns.Value];
                app.Path=p7_make_path(app.UI.pathType.Value,params,app.Raw,c);
                app.Prepared=true; app.showPath();
                lines={'Preview: orange = original stroke, blue = processed reference.', ...
                    'Inspect the shape, then Validate path and Confirm processed path.'};
                if isfield(app.Path,'smoothing')
                    sm=app.Path.smoothing;
                    lines{end+1}=sprintf('Maximum shift %.3f m / %.3f m budget; smoothing span %.2f m.', ...
                        sm.maxDeviation,sm.deviationBudget,sm.spanMetres);
                end
                app.status(lines);
            catch ME
                app.status({ME.message});
            end
        end
        function checkPath(app)
            if app.Running, return; end
            if ~app.Prepared, app.preparePath(); end
            if ~app.Prepared, return; end
            try
                c=app.config();
                app.Report=p7_check_path(app.Path,c); app.Confirmed=false;
                app.showPath();
                app.UI.confirm.Enable=app.onoff(app.Report.ok);
                app.status([{app.Report.summary};app.Report.warnings(:); ...
                    {'Passing this check does not guarantee closed-loop tracking success.'}]);
            catch ME
                app.Confirmed=false; app.Report=[]; app.UI.confirm.Enable='off';
                app.status({ME.message});
            end
        end
        function showPath(app)
            app.resetAxes(); p=app.Path;
            plot(app.Ax,p.raw(:,1),p.raw(:,2),':','Color',[1.0 0.65 0.08],'LineWidth',1.4, ...
                'DisplayName','Original','HitTest','off');
            plot(app.Ax,p.xy(:,1),p.xy(:,2),'-','LineWidth',1.6,'Color',[0.1 0.45 0.75], ...
                'DisplayName','Processed reference','HitTest','off');
            if ~isempty(app.Report) && ~isempty(app.Report.badXY)
                plot(app.Ax,app.Report.badXY(:,1),app.Report.badXY(:,2),'r.', ...
                    'DisplayName','Invalid samples','HitTest','off');
            end
            c=app.config(); plot(app.Ax,c.q0(1),c.q0(2),'ko','DisplayName','Initial true position','HitTest','off');
            box=[p.xy;c.q0(1:2)']; lo=min(box,[],1)-2; hi=max(box,[],1)+2;
            xlim(app.Ax,[lo(1) hi(1)]); ylim(app.Ax,[lo(2) hi(2)]);
            legend(app.Ax,'show','Location','best');
            app.LiveTrue=[]; app.LiveHat=[]; app.LiveRobot=[];
        end
        function confirmPath(app)
            if isempty(app.Report) || ~app.Report.ok, return; end
            app.Confirmed=true;
            app.status({'Path confirmed. Ready to run in Simulink.', ...
                'demo = pursuit test; sf/lqr = nominal feedback; mpc = selected prediction model.', ...
                'Editing any setting invalidates confirmation.'});
        end
        function beginDraw(app)
            if app.Running, return; end
            app.UI.pathType.Value='Freehand'; app.Raw=zeros(0,2); app.Path=[];
            app.invalidate(); app.resetAxes(); legend(app.Ax,'off');
            disableDefaultInteractivity(app.Ax);
            app.DrawLine=plot(app.Ax,NaN,NaN,'-','Color',[1.0 0.65 0.08],'LineWidth',1.8,'HitTest','off','PickableParts','none');
            app.status({'Press and drag inside the plot to draw one continuous path.', ...
                'Each new drag replaces the previous stroke. Smooth / preview, then Validate path when finished.'});
        end
        function mouseDown(app)
            if app.Running || ~strcmp(app.UI.pathType.Value,'Freehand'), return; end
            if ~strcmp(app.Fig.SelectionType,'normal'), return; end
            p=app.Ax.CurrentPoint; p=p(1,1:2);
            if ~app.inside(p), return; end
            app.Prepared=false; app.Confirmed=false; app.Report=[]; app.UI.confirm.Enable='off';
            app.Drawing=true; app.Raw=p;
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
        function randomPose(app)
            if isempty(app.Path), app.checkPath(); end
            if isempty(app.Path), return; end
            stream=RandStream('mt19937ar','Seed',round(app.UI.seed.Value));
            lo=min(app.Path.xy,[],1)-2; hi=max(app.Path.xy,[],1)+2;
            p=lo+(hi-lo).*rand(stream,1,2);
            app.UI.x0.Value=p(1); app.UI.y0.Value=p(2);
            app.UI.theta0.Value=-180+360*rand(stream);
            app.invalidate(); app.showPath();
            app.status({'Random pose set reproducibly from the seed. Change seed for another pose.', ...
                'Smooth / preview and validate and confirm again before running.'});
        end
        function runTest(app)
            if app.Running, return; end
            if ~app.Confirmed
                app.status({'Smooth / preview and validate and confirm the processed path first.'}); return
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
                app.Result=p7_run(app.Path,app.config(),@(s)app.onFrame(s),'simulink');
                if isgraphics(app.Fig)
                    app.showResult();
                end
            catch ME
                if isgraphics(app.Fig), app.status({ME.message}); end
            end
        end
        function onFrame(app,s)
            app.Session=s;
            if app.StopPending, s.CancelRequested=true; end
            if ~isgraphics(app.Fig), s.CancelRequested=true; return; end
            L=s.Log;
            if isempty(app.LiveTrue) || ~isgraphics(app.LiveTrue)
                app.LiveTrue=plot(app.Ax,NaN,NaN,'-','Color',[0.85 0.2 0.15], ...
                    'LineWidth',1.5,'DisplayName','True trajectory','HitTest','off');
                app.LiveHat=plot(app.Ax,NaN,NaN,'--','Color',[0.1 0.6 0.3], ...
                    'DisplayName','Estimated trajectory','HitTest','off');
                app.LiveRobot=quiver(app.Ax,0,0,0,0,0,'k','LineWidth',2, ...
                    'DisplayName','Robot heading','HitTest','off');
            end
            app.LiveTrue.XData=L.qTrue(1,:); app.LiveTrue.YData=L.qTrue(2,:);
            app.LiveHat.XData=L.qHat(1,:); app.LiveHat.YData=L.qHat(2,:);
            q=L.qTrue(:,end);
            set(app.LiveRobot,'XData',q(1),'YData',q(2),'UData',0.5*cos(q(3)),'VData',0.5*sin(q(3)));
            title(app.Ax,sprintf('%s | %s | t = %.1f s',s.Config.controller,s.Config.referenceMode,L.t(end)));
            drawnow limitrate
        end
        function showResult(app)
            r=app.Result; L=r.log; m=r.metrics; n=m.samples;
            if ~isempty(app.Session), app.onFrame(app.Session); end
            if n>0
                plot(app.ErrorAxes,L.t(1:n),m.positionError,'-',L.t(1:n),m.headingError,'--');
                grid(app.ErrorAxes,'on'); xlabel(app.ErrorAxes,'Time (s)');
                legend(app.ErrorAxes,{'Position error (m)','Heading error (rad)'});
                title(app.ErrorAxes,['Errors | ' r.config.referenceMode ' mode']);
                plot(app.InputAxes,L.t(1:n),L.omega(1:n)); grid(app.InputAxes,'on');
                xlabel(app.InputAxes,'Time (s)'); ylabel(app.InputAxes,'Applied turn rate (rad/s)');
                plot(app.TimingAxes,L.t(1:n),L.solveTime(1:n)); hold(app.TimingAxes,'on');
                yline(app.TimingAxes,r.config.Ts,'--','Ts'); hold(app.TimingAxes,'off');
                grid(app.TimingAxes,'on'); xlabel(app.TimingAxes,'Time (s)'); ylabel(app.TimingAxes,'Controller call time (s)');
            end
            app.status({sprintf('Status: %s | %s',r.status,r.message), ...
                sprintf('Position RMSE: %.4f m | Estimate RMSE: %.4f m | Tracking criterion: %d',m.positionRMSE,m.estimationRMSE,m.trackingSuccess), ...
                sprintf('Mean / max controller time: %.5f / %.5f s | Failures: %d',m.meanSolveTime,m.maxSolveTime,m.failureCount), ...
                'A completed simulation is not automatically a successful tracking result. Export preserves this distinction.'});
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
            app.UI.confirm.Enable=app.onoff(~isempty(app.Report) && app.Report.ok);
        end
        function openModel(app)
            try
                file=build_p7_model;
                open_system(file);
            catch ME
                app.status({ME.message});
            end
        end
        function exportResult(app)
            if isempty(app.Result), app.status({'Run a test before exporting.'}); return; end
            try
                folder=p7_export(app.Result,app.Root);
                exportgraphics(app.Ax,fullfile(folder,'trajectory.png'));
                app.status({['Saved result: ' folder]});
            catch ME
                app.status({ME.message});
            end
        end
        function closeApp(app)
            app.stopTest(); delete(app.Fig);
        end
        function status(app,lines)
            if isgraphics(app.Fig), app.UI.status.Value=lines(:); end
        end
    end
    methods (Static)
        function value=onoff(flag)
            if flag, value='on'; else, value='off'; end
        end
    end
end
