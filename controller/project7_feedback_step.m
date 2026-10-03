function [omega,memory,info] = project7_feedback_step(e,wr,config,memory)
% PROJECT7_FEEDBACK_STEP Fixed nominal SF/LQR design with time-varying feedforward.
if isempty(memory) || ~isfield(memory,'design')
    parameters=[];
    if isfield(config,'design'), parameters=config.design; end
    [~,~,d]=project7_q4_gains(parameters,false);
    memory=struct('design',d);
end
switch config.controller
    case 'sf', K=memory.design.K;
    case 'lqr', K=memory.design.Klqr;
    otherwise, error('P7:Controller','Expected sf or lqr.');
end
omega=wr-K*e;
info=struct('ok',isfinite(omega),'message','Fixed nominal feedback with reference feedforward.', ...
    'predictedQ',[],'constraintResidual',NaN);
end
