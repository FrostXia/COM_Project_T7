function [omega,memory,info] = external_controller(qHat,ref,config,memory)
% EXTERNAL_CONTROLLER Connect the platform to the shared MPC solver.
% Default to nonlinear prediction for this GUI bridge only.
if ~isfield(config,'algorithm'), config.algorithm=struct(); end
if ~isfield(config.algorithm,'mode'), config.algorithm.mode='nonlinear'; end
[omega,memory,info] = mpc_controller_adapter(qHat,ref,config,memory);
end
