function [omega,memory,info] = feedback_control_step(e,wr,config,memory)
% FEEDBACK_CONTROL_STEP Fixed nominal SF/LQR design with time-varying feedforward.
if isempty(memory) || ~isfield(memory,'design')
    parameters=[];
    if isfield(config,'design'), parameters=config.design; end
    [~,~,d]=design_feedback_gains(parameters,false);
    memory=struct('design',d);
end
switch config.controller
    case 'sf', K=memory.design.K;
    case 'lqr', K=memory.design.Klqr;
    otherwise, error('Trajectory:Controller','Expected sf or lqr.');
end
omega=wr-K*e;
info=struct('ok',isfinite(omega),'message','Fixed nominal feedback with reference feedforward.', ...
    'predictedQ',[],'constraintResidual',NaN);
end
