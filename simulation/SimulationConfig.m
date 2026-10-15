classdef SimulationConfig
    % SIMULATIONCONFIG Related operations grouped as static methods.
    methods (Static)

        function c = default_config
        % DEFAULT_CONFIG Explicit demonstration settings, not validated project parameters.
        c.v0 = 0.5;                         % Fixed robot speed (m/s)
        c.vr = 0.5;                         % Independent desired trajectory speed (m/s)
        c.Ts = 0.1;
        c.N = 20;
        c.wMin = -1.2;
        c.wMax = 1.2;
        c.q0 = [-1;-1;0.3];
        c.referenceMode = 'time';
        c.controller = 'openloop';
        c.controllerFcn = @ControlDesign.mpc_controller_adapter;
        c.referenceFcn = [];
        c.controlNoiseEnabled = true;
        c.controlStd = 0.03;                 % Maximum control error (rad/s), Gaussian sigma=bound/3
        c.stateNoiseEnabled = true;
        c.stateStd = [0.05;0.05;0.02];       % Measurement-error maxima +/- [X_e(m);Y_e(m);theta_e(rad)], sigma=bound/3
        c.seed = 42;
        c.lookahead = 0.8;
        c.pathSpacing = 0.03;
        c.smoothingWindow = 11;
        c.maxPathDeviation = 0.20;
        c.endpointTolerance = 0.25;
        c.endpointHold = true;
        c.enforcePathTurnLimit = true;
        c.skipPathValidation = false;
        c.spawnFraction = 0.10;
        c.trackingTolerance = 0.3;
        c.successHoldTime = 2;
        c.uiStride = 3;
        c.preset.line.length = 30;           % m
        c.preset.circle.radius = 10;         % m
        c.preset.spiral.initialRadius = 8;   % m
        c.preset.spiral.growthRate = 0.15;    % m/rad
        c.preset.spiral.turns = 5;
        % Code-only control weights and nominal-turn policy.
        [~,~,design] = ControlDesign.design_feedback_gains([],false,true);
        c.design = design.parameters;
        c.design.v0=c.v0; c.design.vr=c.vr;
        c.nominalTurn = struct('thresholdDeg',1,'fallbackDeg',1);
        c.nominalOverride = [];
        c.algorithm = struct('mode','nonlinear','Q',c.design.Q,'R',c.design.R, ...
            'terminalWeight',zeros(3),'maxIterations',100,'tolerance',1e-6,'predictionSubsteps',4);
        end

        function validate_simulation_config(c)
        % VALIDATE_SIMULATION_CONFIG Reject ambiguous or invalid simulation settings.
        validateattributes(c.v0,{'double'},{'scalar','real','finite','positive'});
        SimulationConfig.reference_speed(c);
        validateattributes(c.Ts,{'double'},{'scalar','real','finite','positive'});
        validateattributes(c.N,{'double'},{'scalar','integer','>=',1,'<=',1000});
        validateattributes(c.q0,{'double'},{'size',[3,1],'real','finite'});
        validateattributes(c.stateStd,{'double'},{'size',[3,1],'real','finite','nonnegative'});
        validateattributes(c.seed,{'double'},{'scalar','integer','>=',0,'<',2^32-2});
        validateattributes(c.endpointTolerance,{'double'},{'scalar','finite','positive'});
        validateattributes(c.spawnFraction,{'double'},{'scalar','finite','nonnegative'});
        validateattributes(c.controlStd,{'double'},{'scalar','real','finite','nonnegative'});
        for key={'controlNoiseEnabled','stateNoiseEnabled'}
            validateattributes(c.(key{1}),{'logical','double'},{'scalar','binary'});
        end
        validateattributes(c.lookahead,{'double'},{'scalar','finite','positive'});
        validateattributes(c.pathSpacing,{'double'},{'scalar','finite','positive'});
        validateattributes(c.maxPathDeviation,{'double'},{'scalar','finite','nonnegative'});
        assert(isfinite(c.wMin) && isfinite(c.wMax) && c.wMin < 0 && c.wMax > 0, ...
            'Trajectory:Bounds','Turning bounds must straddle zero.');
        assert(ismember(c.referenceMode,{'path','time'}),'Trajectory:Mode','Invalid reference mode.');
        if ~isempty(c.nominalOverride)
            validateattributes(c.nominalOverride,{'double'},{'scalar','real','finite','nonzero'});
        end
        assert(ismember(c.controller,{'openloop','sf','lqr','mpc','external'}),'Trajectory:Controller','Invalid controller.');
        end

        function vr = reference_speed(c)
        % REFERENCE_SPEED Read the explicitly configured reference speed.
        assert(isstruct(c) && isfield(c,'vr'),'Trajectory:ReferenceSpeed', ...
            'Configuration must include vr. Start from SimulationConfig.default_config.');
        vr=c.vr;
        validateattributes(vr,{'double'},{'scalar','real','finite','positive'});
        end
    end
end
