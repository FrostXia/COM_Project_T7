classdef P7Session < handle
    % P7SESSION Own one run; algorithms only receive their documented inputs.
    properties
        Config
        AlgorithmConfig
        Path
        Log
        EstimatorMemory = []
        ControllerMemory = []
        ReferenceMemory = []
        PlantStream
        SensorStream
        ObserverIndex = -1
        ControllerIndex = -1
        CachedEstimate
        CachedMeasurement
        CachedOmega = 0
        CancelRequested = false
        StopRequested = false
        Status = 'running'
        Message = ''
        Callback = []
        LastInfo = struct()
    end
    methods
        function obj=P7Session(path,c,callback)
            p7_validate_config(c);
            obj.Config=c; obj.Path=path;
            obj.AlgorithmConfig=rmfield(c,{'q0','initialEstimateOffset'});
            obj.AlgorithmConfig.qHat0=c.q0+c.initialEstimateOffset;
            if nargin>2, obj.Callback=callback; end
            obj.PlantStream=RandStream('mt19937ar','Seed',c.seed);
            obj.SensorStream=RandStream('mt19937ar','Seed',c.seed+1);
            obj.Log=struct('t',[],'qTrue',zeros(3,0),'measurement',zeros(3,0), ...
                'qHat',zeros(3,0),'reference',zeros(3,0),'progress',[], ...
                'omega',[],'omegaRequested',[],'omegaReference',[], ...
                'solveTime',[],'ok',[],'constraintResidual',[],'messages',{{}});
        end
        function [hat,measurement]=observe(obj,t,qTrue,previousOmega)
            c=obj.Config; k=round(t/c.Ts)+1;
            if obj.ObserverIndex==k
                hat=obj.CachedEstimate; measurement=obj.CachedMeasurement; return
            end
            ny=2+double(strcmp(c.sensorMode,'pose'));
            y=qTrue(1:ny)+double(c.measurementNoiseEnabled)*c.measurementStd(1:ny).*randn(obj.SensorStream,ny,1);
            if ny==3, y(3)=p7_wrap(y(3)); end
            if strcmp(c.estimatorMode,'ideal')
                hat=qTrue;
            else
                [hat,obj.EstimatorMemory]=p7_estimator(y,previousOmega,obj.AlgorithmConfig,obj.EstimatorMemory);
            end
            validateattributes(hat,{'double'},{'size',[3 1],'real','finite'});
            measurement=nan(3,1); measurement(1:ny)=y;
            obj.Log.t(k)=t; obj.Log.qTrue(:,k)=qTrue;
            obj.Log.qHat(:,k)=hat; obj.Log.measurement(:,k)=measurement;
            obj.CachedEstimate=hat; obj.CachedMeasurement=measurement; obj.ObserverIndex=k;
        end
        function w=control(obj,t,hat)
            c=obj.Config; k=round(t/c.Ts)+1;
            if obj.ControllerIndex==k, w=obj.CachedOmega; return; end
            if obj.CancelRequested
                obj.Status='cancelled'; obj.Message='Stopped by the user.'; obj.StopRequested=true;
            end
            [ref,obj.ReferenceMemory]=p7_reference(hat,t,obj.Path,obj.AlgorithmConfig,obj.ReferenceMemory);
            info=struct('ok',true,'message','','predictedQ',[],'constraintResidual',NaN);
            requested=0;
            elapsed=0;
            if ~obj.StopRequested
                started=tic;
                try
                    switch c.controller
                        case 'demo'
                            [requested,obj.ControllerMemory,info]=p7_demo_controller(hat,ref,obj.AlgorithmConfig,obj.ControllerMemory);
                        case 'openloop'
                            requested=ref.w(1);
                            info.message='Open-loop reference turn rate.';
                        case 'external'
                            [requested,obj.ControllerMemory,info]=c.controllerFcn(hat,ref,obj.AlgorithmConfig,obj.ControllerMemory);
                    end
                    assert(isstruct(info) && isfield(info,'ok') && isscalar(info.ok) && ...
                        (islogical(info.ok) || (isnumeric(info.ok) && isfinite(info.ok) && ismember(info.ok,[0 1]))), ...
                        'P7:Controller','info.ok must be a logical scalar.');
                    if info.ok
                        assert(isscalar(requested) && isnumeric(requested) && isreal(requested) && isfinite(requested), ...
                            'P7:Controller','Controller output must be a finite real scalar.');
                    end
                catch ME
                    info=struct('ok',false,'message',ME.message,'predictedQ',[],'constraintResidual',NaN);
                    requested=NaN;
                end
                elapsed=toc(started);
            end
            if ~isfield(info,'message'), info.message=''; end
            if ~isfield(info,'constraintResidual'), info.constraintResidual=NaN; end
            if ~info.ok
                obj.Status='failed'; obj.Message=char(info.message); obj.StopRequested=true;
            end
            if obj.StopRequested
                w=0;
            else
                w=min(max(requested,c.wMin),c.wMax);
                if strcmp(c.referenceMode,'path') && ...
                        ref.s>=obj.Path.length-c.endpointTolerance && ...
                        norm(hat(1:2)-obj.Path.xy(end,:)')<=c.endpointTolerance
                    obj.Status='endpoint'; obj.Message='Estimated pose reached the path endpoint region.';
                    obj.StopRequested=true;
                end
            end
            obj.Log.reference(:,k)=ref.q(:,1); obj.Log.progress(k)=ref.s;
            obj.Log.omega(k)=w; obj.Log.omegaRequested(k)=requested;
            obj.Log.omegaReference(k)=ref.w(1); obj.Log.solveTime(k)=elapsed;
            obj.Log.ok(k)=logical(info.ok); obj.Log.constraintResidual(k)=info.constraintResidual;
            obj.Log.messages{k}=char(info.message); obj.LastInfo=info;
            obj.ControllerIndex=k; obj.CachedOmega=w;
            if ~isempty(obj.Callback) && (mod(k-1,c.uiStride)==0 || obj.StopRequested)
                obj.Callback(obj);
            end
            if obj.CancelRequested && ~obj.StopRequested
                obj.Status='cancelled'; obj.Message='Stopped by the user.'; obj.StopRequested=true;
            end
        end
        function q=advance(obj,q,w)
            c=obj.Config;
            q=p7_motion(q,w,c);
            q=q+double(c.stateNoiseEnabled)*c.stateStd.*randn(obj.PlantStream,3,1);
        end
    end
end
