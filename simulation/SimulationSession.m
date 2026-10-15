classdef SimulationSession < handle
    % SIMULATIONSESSION Own one run; algorithms only receive their documented inputs.
    properties
        Config
        AlgorithmConfig
        Path
        Log
        ControllerMemory = []
        ReferenceMemory = []
        MeasurementStream
        ControlStream
        CachedControlNoise = 0
        ControllerIndex = -1
        CachedOmega = 0
        CancelRequested = false
        StopRequested = false
        Status = 'running'
        Message = ''
        Callback = []
        LastInfo = struct()
    end
    methods
        function obj=SimulationSession(path,c,callback)
            SimulationConfig.validate_simulation_config(c);
            obj.Config=c; obj.Path=path;
            obj.AlgorithmConfig=rmfield(c,'q0');
            if nargin>2, obj.Callback=callback; end
            obj.MeasurementStream=RandStream('mt19937ar','Seed',c.seed);
            obj.ControlStream=RandStream('mt19937ar','Seed',c.seed+2);
            obj.Log=struct('t',[],'qTrue',zeros(3,0),'trackingError',zeros(3,0), ...
                'stateNoise',zeros(3,0),'qMeasured',zeros(3,0),'measuredTrackingError',zeros(3,0), ...
                'reference',zeros(3,0),'progress',[], ...
                'omega',[],'omegaPhysical',[],'controlNoise',[],'controlNoiseEffective',[], ...
                'omegaRequested',[],'omegaReference',[], ...
                'solveTime',[],'ok',[],'constraintResidual',[],'messages',{{}});
        end
        function w=control(obj,t,qTrue)
            c=obj.Config; k=round(t/c.Ts)+1;
            if obj.ControllerIndex==k, w=obj.CachedOmega; return; end
            validateattributes(qTrue,{'double'},{'size',[3 1],'real','finite'});
            obj.Log.t(k)=t; obj.Log.qTrue(:,k)=qTrue;
            if obj.CancelRequested
                obj.Status='cancelled'; obj.Message='Stopped by the user.'; obj.StopRequested=true;
            end
            % Fix one reference, then sample measurement error in tracking coordinates.
            [ref,obj.ReferenceMemory]=TrajectoryPath.sample_reference_preview(qTrue,t,obj.Path,obj.AlgorithmConfig,obj.ReferenceMemory);
            validateattributes(ref.q,{'double'},{'size',[3 c.N+1],'real','finite'});
            trueError=RobotDynamics.tracking_error_state(qTrue,ref.q(:,1));
            measurementNoise=zeros(3,1);
            if ~obj.StopRequested && c.stateNoiseEnabled
                measurementNoise=RobotDynamics.sample_truncated_gaussian(obj.MeasurementStream,c.stateStd/3,c.stateStd);
            end
            ref.measuredTrackingError=trueError+measurementNoise;
            % Pose-only external interfaces receive an equivalent measured pose.
            % Built-in controllers use the exact noisy error, including its angle branch.
            qMeasured=qTrue;
            if any(measurementNoise~=0)
                theta=qTrue(3)-measurementNoise(3);
                rotation=[cos(theta) -sin(theta);sin(theta) cos(theta)];
                qMeasured=[ref.q(1:2,1)-rotation*ref.measuredTrackingError(1:2);theta];
            end
            obj.Log.stateNoise(:,k)=measurementNoise;
            obj.Log.qMeasured(:,k)=qMeasured;
            info=struct('ok',true,'message','','predictedQ',[],'constraintResidual',NaN);
            requested=0;
            elapsed=0;
            if ~obj.StopRequested
                started=tic;
                try
                    switch c.controller
                        case 'openloop'
                            requested=ref.w(1);
                            info.message='Open-loop reference turn rate.';
                        case {'sf','lqr','mpc'}
                            [requested,obj.ControllerMemory,info]=ControlDesign.feedback_controller_adapter(qMeasured,ref,obj.AlgorithmConfig,obj.ControllerMemory);
                        case 'external'
                            [requested,obj.ControllerMemory,info]=c.controllerFcn(qMeasured,ref,obj.AlgorithmConfig,obj.ControllerMemory);
                    end
                    assert(isstruct(info) && isfield(info,'ok') && isscalar(info.ok) && ...
                        (islogical(info.ok) || (isnumeric(info.ok) && isfinite(info.ok) && ismember(info.ok,[0 1]))), ...
                        'Trajectory:Controller','info.ok must be a logical scalar.');
                    if info.ok
                        assert(isscalar(requested) && isnumeric(requested) && isreal(requested) && isfinite(requested), ...
                            'Trajectory:Controller','Controller output must be a finite real scalar.');
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
            end
            obj.Log.reference(:,k)=ref.q(:,1); obj.Log.progress(k)=ref.s;
            obj.Log.trackingError(:,k)=trueError;
            obj.Log.measuredTrackingError(:,k)=ref.measuredTrackingError;
            % Draw one input disturbance per control sample before physical clipping.
            noise=0;
            if ~obj.StopRequested && c.controlNoiseEnabled
                noise=RobotDynamics.sample_truncated_gaussian(obj.ControlStream,c.controlStd/3,c.controlStd);
            end
            physical=min(max(w+noise,c.wMin),c.wMax);
            obj.CachedControlNoise=noise;
            obj.Log.omega(k)=w; obj.Log.omegaPhysical(k)=physical;
            obj.Log.controlNoise(k)=noise; obj.Log.controlNoiseEffective(k)=physical-w;
            obj.Log.omegaRequested(k)=requested;
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
            assert(obj.ControllerIndex>=1,'Trajectory:StepOrder','Call control before advancing the plant.');
            if obj.StopRequested, return; end
            % Do not advance a full interval outside this run.
            if isfield(c,'desiredDuration') && obj.ControllerIndex*c.Ts>c.desiredDuration+1e-10
                return
            end
            physical=min(max(w+obj.CachedControlNoise,c.wMin),c.wMax);
            q=RobotDynamics.integrate_robot_motion(q,physical,c);
            % Measurement noise is drawn in control; no state jump is applied here.
        end
    end
end
