function T = evaluate_experiment(result)
% EVALUATE_EXPERIMENT Export only metrics whose evaluation rules are confirmed.
L=result.log; c=result.config;
n=min([numel(L.t),size(L.qTrue,2),size(L.qHat,2),size(L.reference,2), ...
    numel(L.omega),numel(L.solveTime),numel(L.ok)]);
names=strings(0,1); values=names; units=names;
add('Controller',c.controller,'');
add('Engine',result.engine,'');
add('Run Status',result.status,'');
add('Stop Reason',result.message,'');
add('Trajectory Type',result.path.kind,'');
add('Path Length',result.path.length,'m');
add('Desired Trajectory Speed',c.v0,'m/s');
referenceEnd=result.path.length/c.v0;
add('Desired Duration',referenceEnd,'s');
add('Sample Period',c.Ts,'s');
add('Maximum Omega',c.wMax,'rad/s');
add('Minimum Omega',c.wMin,'rad/s');
add('Robot Initial Pose',c.q0,'[m m rad]');
add('Initial Estimate Offset',c.initialEstimateOffset,'[m m rad]');
add('Reference Mode',c.referenceMode,'');
add('Estimator Mode',c.estimatorMode,'');
add('Sensor Mode',c.sensorMode,'');
add('Random Seed',c.seed,'');
add('Measurement Noise Enabled',c.measurementNoiseEnabled,'');
add('Measurement Noise Standard Deviation',c.measurementStd,'[m m rad]');
add('Control Noise Enabled',c.controlNoiseEnabled,'');
add('Control Noise Standard Deviation',c.controlStd,'rad/s');
add('State Noise Enabled',c.stateNoiseEnabled,'');
add('State Noise Standard Deviation',c.stateStd,'[m m rad] per update');
add('Estimator Qn',c.estimatorQn,'');
add('Estimator Rn',c.estimatorRn,'');
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
pathRms=NaN; pathMax=NaN; estimatePosition=NaN; estimateHeading=NaN;
turnMax=NaN; variation=NaN; saturation=NaN;
meanTime=NaN; maxTime=NaN; missRate=NaN; failures=NaN;
if n>0
    t=L.t(1:n); duration=t(end)-t(1);
    covered=100*min(max(t(end),0),referenceEnd)/referenceEnd;
    q=L.qTrue(:,1:n); ref=L.reference(:,1:n); hat=L.qHat(:,1:n);
    e=vecnorm(q(1:2,:)-ref(1:2,:),2,1);
    h=rad2deg(wrap_angle(q(3,:)-ref(3,:)));
    positionRms=sqrt(mean(e.^2)); positionMax=max(e);
    headingRms=sqrt(mean(h.^2)); headingMax=max(abs(h));
    distances=segmentDistances(q(1:2,:).',result.path.xy);
    pathRms=sqrt(mean(distances.^2)); pathMax=max(distances);
    estimatePosition=sqrt(mean(sum((q(1:2,:)-hat(1:2,:)).^2,1)));
    estimateHeading=sqrt(mean(rad2deg(wrap_angle(q(3,:)-hat(3,:))).^2));
    finalDistance=norm(q(1:2,end)-result.path.xy(end,:).');
    w=L.omega(1:n);
    if isfield(L,'omegaPhysical'), w=L.omegaPhysical(1:n); end
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
add('Position Estimation Root Mean Square Error',estimatePosition,'m');
add('Heading Estimation Root Mean Square Error',estimateHeading,'deg');
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
