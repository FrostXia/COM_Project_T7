function c=defaults
% DEFAULTS Common experiment settings; algorithm parameters come from Task 1.
%% 1. Fixed design parameters and physical/reference speeds
[~,~,d]=project7_q4_gains([],false,true);
c.design=d.parameters;
c.v0=d.parameters.v0; c.vr=c.v0;
c.Ts=0.1; c.N=20; c.duration=30;
c.q0=[0;0;0]; c.qr0=[0;0;0];
c.referenceKind='q4circle';
c.pathParams=[12 3 0.15 1];
c.integrationSteps=100;
%% 2. Noise conventions: independent measurement and angular-rate disturbance
c.measurementStd=[0.05;0.05;0.05];
c.controlStd=0.1;
c.noiseAtInitialSample=false; % Match the uncommented Q5 initial state
c.seed=42;
c.scenarios={'clean','observer','control','both'};
c.wMin=-1.2; c.wMax=1.2;
c.applyCommandLimits=true;
% Noise is added AFTER command limiting; the disturbed physical rate is not clipped.
%% 3. Optional error-state Kalman filter; raw measurements are the Q5 default
c.filter.mode='none'; % 'none' or 'kf'
c.filter.Qn=[]; % USER TO FILL: scalar angular-rate noise variance
c.filter.Rn=[]; % USER TO FILL: 3-by-3 error-measurement covariance
c.filter.qScale=1; c.filter.rScale=1;
c.filter.initialMean=zeros(3,1);
c.filter.P0=diag([0.1 0.1 0.1].^2);
%% 4. MPC setup: LTV by default; optional nonlinear error prediction
c.mpc.mode='ltv'; % 'ltv' or 'nonlinear'
c.mpc.Q=d.parameters.Q; c.mpc.R=d.parameters.R;
c.mpc.terminalWeight=zeros(3);
c.mpc.maxIterations=100;
c.mpc.tolerance=1e-6;
c.mpc.predictionSubsteps=4;
%% 5. Path preparation, evaluation and output
c.pathSpacing=0.03; c.smoothingWindow=11;
c.maxPathDeviation=0.20; c.lookahead=0.8;
c.analysisStart=0; % All samples; change explicitly for a tail-window study
c.plot=true; c.save=true;
c.outputRoot=fullfile(pwd,'results');
end
