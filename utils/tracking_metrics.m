function m = tracking_metrics(result)
% TRACKING_METRICS Distinguish path distance, time error and estimator error.
L=result.log; c=result.config;
n=min([numel(L.t),size(L.reference,2),size(L.qTrue,2),numel(L.omega)]);
m=struct('samples',n,'positionRMSE',NaN,'headingRMSE',NaN,'estimationRMSE',NaN, ...
    'meanSolveTime',NaN,'maxSolveTime',NaN,'controlEffort',NaN, ...
    'saturationRatio',NaN,'peakTurnRate',NaN,'convergenceTime',NaN, ...
    'failureCount',0,'overrunCount',0,'trackingSuccess',false, ...
    'referenceEndTime',result.path.length/c.v0,'endpointReached',false, ...
    'endpointArrivalTime',NaN,'finalEndpointDistance',NaN);
if n==0, return; end
qt=L.qTrue(:,1:n); qr=L.reference(:,1:n);
if strcmp(c.referenceMode,'path')
    % Score against the selected path neighbourhood to avoid crossing jumps.
    for k=1:n
        ids=find(abs(result.path.s-L.progress(k))<=max(2*c.lookahead,2*c.v0*c.Ts));
        if isempty(ids), [~,ids]=min(abs(result.path.s-L.progress(k))); end
        [~,j]=min(sum((result.path.xy(ids,:)-qt(1:2,k)').^2,2));
        idx=ids(j); qr(:,k)=[result.path.xy(idx,:)';result.path.theta(idx)];
    end
end
e=vecnorm(qt(1:2,:)-qr(1:2,:),2,1);
h=wrap_angle(qt(3,:)-qr(3,:));
m.positionRMSE=sqrt(mean(e.^2)); m.headingRMSE=sqrt(mean(h.^2));
m.estimationRMSE=sqrt(mean(sum((qt(1:2,:)-L.qHat(1:2,1:n)).^2,1)));
m.meanSolveTime=mean(L.solveTime(1:n)); m.maxSolveTime=max(L.solveTime(1:n));
executed=1:max(0,n-1);
m.controlEffort=c.Ts*sum((L.omega(executed)-L.omegaReference(executed)).^2);
physical=L.omega;
if isfield(L,'omegaPhysical'), physical=L.omegaPhysical; end
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
