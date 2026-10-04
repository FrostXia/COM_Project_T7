function c = default_config
% DEFAULT_CONFIG Explicit demonstration settings, not validated project parameters.
c.v0 = 0.5;
c.Ts = 0.1;
c.N = 20;
c.wMin = -1.2;
c.wMax = 1.2;
c.q0 = [-1;-1;0.3];
c.initialEstimateOffset = [0.1;-0.1;0.1];
c.P0 = diag([0.2,0.2,0.3].^2);
c.referenceMode = 'time';
c.estimatorMode = 'ekf';
c.sensorMode = 'position';
c.controller = 'openloop';
c.controllerFcn = @external_controller;
c.estimatorFcn = [];
c.referenceFcn = [];
c.controlNoiseEnabled = true;
c.controlStd = 0.1;
c.stateNoiseEnabled = false;
c.measurementNoiseEnabled = true;
c.stateStd = [0.003;0.003;0.005];
c.measurementStd = [0.03;0.03;0.03];
c.seed = 42;
c.lookahead = 0.8;
c.pathSpacing = 0.03;
c.smoothingWindow = 11;
c.maxPathDeviation = 0.20;
c.endpointTolerance = 0.25;
c.endpointHold = true;
c.spawnFraction = 0.10;
c.trackingTolerance = 0.3;
c.successHoldTime = 2;
c.uiStride = 3;
c.measurementFloor = 1e-10;
% Optional fixed EKF covariances; empty retains code-derived noise covariances.
c.estimatorQn = [];
c.estimatorRn = [];
c.preset.line.length = 30;           % m
c.preset.circle.radius = 10;         % m
c.preset.spiral.initialRadius = 8;   % m
c.preset.spiral.growthRate = 0.15;    % m/rad
c.preset.spiral.turns = 5;
% Code-only control weights, estimation and nominal-turn policy.
[~,~,design] = design_feedback_gains([],false,true);
c.design = design.parameters;
c.nominalTurn = struct('thresholdDeg',1,'fallbackDeg',1);
c.nominalOverride = [];
c.algorithm = struct('mode','nonlinear','Q',c.design.Q,'R',c.design.R, ...
    'terminalWeight',zeros(3),'maxIterations',100,'tolerance',1e-6,'predictionSubsteps',4);
end
