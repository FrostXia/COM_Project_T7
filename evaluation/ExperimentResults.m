classdef ExperimentResults
    % EXPERIMENTRESULTS Related operations grouped as static methods.
    methods (Static)

        function m = tracking_metrics(result)
        % TRACKING_METRICS Evaluate true path distance and time-referenced tracking error.
        L=result.log; c=result.config;
        n=min([numel(L.t),size(L.reference,2),size(L.qTrue,2),numel(L.omega)]);
        m=struct('samples',n,'positionRMSE',NaN,'headingRMSE',NaN, ...
            'meanSolveTime',NaN,'maxSolveTime',NaN,'controlEffort',NaN, ...
            'saturationRatio',NaN,'peakTurnRate',NaN,'convergenceTime',NaN, ...
            'failureCount',0,'overrunCount',0,'trackingSuccess',false, ...
            'referenceEndTime',result.path.length/SimulationConfig.reference_speed(c),'endpointReached',false, ...
            'endpointArrivalTime',NaN,'finalEndpointDistance',NaN);
        if n==0, return; end
        qt=L.qTrue(:,1:n); qr=L.reference(:,1:n);
        if strcmp(c.referenceMode,'path')
            % Score against the selected path neighbourhood to avoid crossing jumps.
            for k=1:n
                ids=find(abs(result.path.s-L.progress(k))<=max(2*c.lookahead,2*SimulationConfig.reference_speed(c)*c.Ts));
                if isempty(ids), [~,ids]=min(abs(result.path.s-L.progress(k))); end
                [~,j]=min(sum((result.path.xy(ids,:)-qt(1:2,k)').^2,2));
                idx=ids(j); qr(:,k)=[result.path.xy(idx,:)';result.path.theta(idx)];
            end
        end
        e=vecnorm(qt(1:2,:)-qr(1:2,:),2,1);
        h=RobotDynamics.wrap_angle(qt(3,:)-qr(3,:));
        m.positionRMSE=sqrt(mean(e.^2)); m.headingRMSE=sqrt(mean(h.^2));
        m.meanSolveTime=mean(L.solveTime(1:n)); m.maxSolveTime=max(L.solveTime(1:n));
        executed=1:max(0,n-1);
        m.controlEffort=c.Ts*sum((L.omega(executed)-L.omegaReference(executed)).^2);
        physical=L.omegaPhysical;
        m.saturationRatio=mean(physical(executed)<=c.wMin+1e-8 | physical(executed)>=c.wMax-1e-8);
        m.failureCount=sum(~L.ok(1:n)); m.overrunCount=sum(L.solveTime(1:n)>c.Ts);
        holdSamples=max(2,ceil(c.successHoldTime/c.Ts)+1);
        m.peakTurnRate=max(abs(physical(1:n)));
        lastOutside=find(e>c.trackingTolerance,1,'last');
        if isempty(lastOutside), firstInside=1; else, firstInside=lastOutside+1; end
        if n-firstInside+1>=holdSamples
            m.convergenceTime=L.t(firstInside);
        end
        m.trackingSuccess=n>=holdSamples && ...
            all(e(end-holdSamples+1:end)<=c.trackingTolerance) && ...
            strcmp(result.status,'completed') && m.failureCount==0;
        endpointDistance=vecnorm(qt(1:2,:)-result.path.xy(end,:).',2,1);
        m.finalEndpointDistance=endpointDistance(end);
        arrival=find(L.t(1:n)>=m.referenceEndTime-1e-10 & endpointDistance<=c.endpointTolerance,1);
        if ~isempty(arrival)
            m.endpointReached=true; m.endpointArrivalTime=L.t(arrival);
        end
        m.positionError=e; m.headingError=h; m.metricReference=qr;
        end

        function T = evaluate_experiment(result)
        % EVALUATE_EXPERIMENT Export only metrics whose evaluation rules are confirmed.
        L=result.log; c=result.config;
        n=min([numel(L.t),size(L.qTrue,2),size(L.reference,2), ...
            numel(L.omega),numel(L.solveTime),numel(L.ok)]);
        names=strings(0,1); values=names; units=names;
        add('Controller',c.controller,'');
        add('Engine',result.engine,'');
        add('Run Status',result.status,'');
        add('Stop Reason',result.message,'');
        add('Trajectory Type',result.path.kind,'');
        add('Path Length',result.path.length,'m');
        add('Desired Trajectory Speed',SimulationConfig.reference_speed(c),'m/s');
        add('Robot Speed',c.v0,'m/s');
        referenceEnd=result.path.length/SimulationConfig.reference_speed(c);
        add('Desired Duration',referenceEnd,'s');
        add('Sample Period',c.Ts,'s');
        add('Maximum Omega',c.wMax,'rad/s');
        add('Minimum Omega',c.wMin,'rad/s');
        add('Robot Initial Pose',c.q0,'[m m rad]');
        add('Reference Mode',c.referenceMode,'');
        add('Feedback','Common measured tracking error','');
        add('Measurement Noise Coordinates','Body-frame tracking error [X_e Y_e theta_e]','');
        add('Measurement Noise Distribution','Gaussian sigma=max/3; reject samples outside +/-max','');
        add('Control Noise Distribution','Gaussian sigma=max/3; reject samples outside +/-max','');
        add('Control Gaussian Sigma',c.controlStd/3,'rad/s');
        add('Measurement Gaussian Sigma',c.stateStd/3,'[m m rad]');
        add('Random Seed',c.seed,'');
        add('Control Noise Enabled',c.controlNoiseEnabled,'');
        add('Control Noise Bound (+/-)',c.controlStd,'rad/s');
        add('Measurement Noise Enabled',c.stateNoiseEnabled,'');
        add('Measurement Noise Bounds (+/-)',c.stateStd,'[m m rad]');
        if ismember(c.controller,{'sf','lqr'})
            fields=fieldnames(c.design);
            for j=1:numel(fields)
                add(['Controller Parameter: ' fields{j}],c.design.(fields{j}),'');
            end
        elseif strcmp(c.controller,'mpc')
            add('MPC Preview Steps',c.N,'');
            fields=fieldnames(c.algorithm);
            for j=1:numel(fields)
                add(['MPC Parameter: ' fields{j}],c.algorithm.(fields{j}),'');
            end
        end
        add('Recorded Sample Count',n,'');
        duration=NaN; covered=NaN; finalDistance=NaN;
        positionRms=NaN; positionMax=NaN; headingRms=NaN; headingMax=NaN;
        pathRms=NaN; pathMax=NaN;
        turnMax=NaN; variation=NaN; saturation=NaN;
        meanTime=NaN; maxTime=NaN; missRate=NaN; failures=NaN;
        if n>0
            t=L.t(1:n); duration=t(end)-t(1);
            covered=100*min(max(t(end),0),referenceEnd)/referenceEnd;
            q=L.qTrue(:,1:n); ref=L.reference(:,1:n);
            e=vecnorm(q(1:2,:)-ref(1:2,:),2,1);
            h=rad2deg(RobotDynamics.wrap_angle(q(3,:)-ref(3,:)));
            positionRms=sqrt(mean(e.^2)); positionMax=max(e);
            headingRms=sqrt(mean(h.^2)); headingMax=max(abs(h));
            distances=segmentDistances(q(1:2,:).',result.path.xy);
            pathRms=sqrt(mean(distances.^2)); pathMax=max(distances);
            finalDistance=norm(q(1:2,end)-result.path.xy(end,:).');
            w=L.omegaPhysical(1:n);
            turnMax=max(abs(w)); variation=sum(abs(diff(w)));
            if n>1 && duration>0
                active=w(1:end-1)<=c.wMin+1e-8 | w(1:end-1)>=c.wMax-1e-8;
                saturation=100*sum(diff(t).*active)/duration;
            end
            meanTime=mean(L.solveTime(1:n)); maxTime=max(L.solveTime(1:n));
            missRate=100*mean(L.solveTime(1:n)>c.Ts); failures=sum(~L.ok(1:n));
        end
        add('Recorded Sample Duration',duration,'s');
        add('Reference Duration Covered',covered,'%');
        add('Position Root Mean Square Error',positionRms,'m');
        add('Maximum Position Error',positionMax,'m');
        add('Heading Root Mean Square Error',headingRms,'deg');
        add('Maximum Absolute Heading Error',headingMax,'deg');
        add('Path Distance Root Mean Square Error',pathRms,'m');
        add('Maximum Path Distance',pathMax,'m');
        add('Final Endpoint Distance',finalDistance,'m');
        add('Maximum Absolute Angular Velocity',turnMax,'rad/s');
        add('Angular Velocity Total Variation',variation,'rad/s');
        add('Angular Velocity Saturation Time',saturation,'%');
        add('Mean Controller Computation Time',meanTime,'s');
        add('Maximum Controller Computation Time',maxTime,'s');
        add('Computation Deadline Miss Rate',missRate,'%');
        if strcmp(c.controller,'mpc')
            add('MPC Solver Failure Count',failures,'');
        else
            add('Controller Failure Count',failures,'');
        end
        T=table(names,values,units,'VariableNames',{'Metric','Value','Unit'});
            function add(name,value,unit)
                names(end+1,1)=string(name);
                if isnumeric(value) || islogical(value)
                    values(end+1,1)=string(mat2str(value,17));
                else
                    values(end+1,1)=string(value);
                end
                units(end+1,1)=string(unit);
            end
        end

        function folder = export_experiment(result,root)
        % EXPORT_EXPERIMENT Export the desired path, evaluation and four plotted data sets.
        folder=fullfile(root,'results');
        if ~isfolder(folder), mkdir(folder); end
        prefixes={'trajectory_','evaluation_','robot_trajectory_','errors_', ...
            'angular_velocity_','controller_timing_'};
        stamp=char(datetime('now','Format','yyyyMMdd_HHmmss_SSS'));
        base=stamp; suffix=0;
        while any(cellfun(@(prefix)isfile(fullfile(folder,[prefix stamp '.csv'])),prefixes))
            suffix=suffix+1; stamp=sprintf('%s_%d',base,suffix);
        end

        % Match the plotted sample interval and true-error definitions in showResult.
        L=result.log; m=ExperimentResults.tracking_metrics(result); n=m.samples;
        t=reshape(L.t(1:n),[],1);
        robot=table(t,L.qTrue(1,1:n).',L.qTrue(2,1:n).',L.qTrue(3,1:n).', ...
            L.reference(1,1:n).',L.reference(2,1:n).',L.reference(3,1:n).', ...
            'VariableNames',{'Time_s','RobotXTrue_m','RobotYTrue_m','RobotHeadingTrue_rad', ...
            'ReferenceX_m','ReferenceY_m','ReferenceHeading_rad'});
        if n==0
            positionError=zeros(0,1); headingError=zeros(0,1);
        else
            positionError=m.positionError(:); headingError=m.headingError(:);
        end
        errors=table(t,positionError,headingError, ...
            'VariableNames',{'Time_s','PositionError_m','HeadingError_rad'});
        angularVelocity=table(t,reshape(L.omegaPhysical(1:n),[],1), ...
            'VariableNames',{'Time_s','ActualAngularVelocity_rad_s'});
        timing=table(t,reshape(L.solveTime(1:n),[],1),repmat(result.config.Ts,n,1), ...
            'VariableNames',{'Time_s','ControllerCallTime_s','SamplePeriod_s'});
        evaluation=ExperimentResults.evaluate_experiment(result);
        evaluation=[table("Export Timestamp",string(stamp),"", ...
            'VariableNames',evaluation.Properties.VariableNames);evaluation];

        % Headerless XY remains compatible with Import Existing Trajectory.
        writematrix(result.path.xy,fullfile(folder,['trajectory_' stamp '.csv']));
        tables={evaluation,robot,errors,angularVelocity,timing};
        for k=1:numel(tables)
            writetable(tables{k},fullfile(folder,[prefixes{k+1} stamp '.csv']),'Encoding','UTF-8');
        end
        end
    end
end

function distance = segmentDistances(points,path)
% Project onto finite segments, including zero-length segments and crossings.
a=path(1:end-1,:); v=diff(path,1,1); lengthSquared=sum(v.^2,2);
distance=zeros(size(points,1),1);
for k=1:size(points,1)
    offset=points(k,:)-a;
    fraction=min(max(sum(offset.*v,2)./max(lengthSquared,eps),0),1);
    residual=offset-fraction.*v;
    distance(k)=sqrt(min(sum(residual.^2,2)));
end
end
