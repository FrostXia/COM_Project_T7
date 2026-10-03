function result=run_mpc_example(c,inputPath)
% RUN_MPC_EXAMPLE Compatibility entry for the original-template MPC runner.
addpath(fileparts(fileparts(fileparts(mfilename('fullpath')))));
setup_project7;
if nargin<1, c=[]; end
if nargin<2, inputPath=[]; end
result=launch_Project7_MPC(c,inputPath);
end
