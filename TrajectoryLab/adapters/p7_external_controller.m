function [omega,memory,info] = p7_external_controller(qHat,ref,config,memory) %#ok<INUSD>
% P7_EXTERNAL_CONTROLLER Replace this body with the teammate's controller.
% qHat: 3-by-1 estimated global pose [x; y; theta], SI units.
% ref.q: 3-by-(N+1), ref.w: N-by-1, ref.path: arc-length geometry.
% config: common configuration; config.algorithm may hold private parameters.
% memory: empty on the first call; store warm starts and progress here.
% omega: scalar physical turn rate, not omega minus reference turn rate.
% info.ok: required logical scalar. Set false to stop and record a failed run.
% info.message: diagnostic text. info.predictedQ: optional 3-by-M preview.
% info.constraintResidual: optional scalar for diagnostics.
% For time tracking, adapt ref.q/ref.w to the teammate's existing functions.
% For path following, implement progress dynamics using ref.path and memory.
% Never read ground truth from a workspace, session, or global variable.
omega = NaN;
info = struct('ok',false,'message', ...
    'External controller is not connected. Implement adapters/p7_external_controller.m.', ...
    'predictedQ',[],'constraintResidual',NaN);
end
