function [omega,memory,info] = p7_external_controller(qHat,ref,config,memory)
% P7_EXTERNAL_CONTROLLER Connect the platform to the shared MPC solver.
% Default to nonlinear prediction for this GUI bridge only.
if ~isfield(config,'algorithm'), config.algorithm=struct(); end
if ~isfield(config.algorithm,'mode'), config.algorithm.mode='nonlinear'; end
[omega,memory,info] = project7_mpc_step(qHat,ref,config,memory);
end
