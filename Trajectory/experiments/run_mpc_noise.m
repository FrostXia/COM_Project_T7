function runs=run_mpc_noise(c,inputPath)
% RUN_MPC_NOISE Task 3B: same Q5 noise channels, separate MPC package.
addpath(fileparts(fileparts(fileparts(mfilename('fullpath')))));
setup_project7;
if nargin<1 || isempty(c), c=p7utils.defaults; end
if nargin<2, inputPath=[]; end
runs=p7utils.run_cases(c,inputPath,{'mpc'},'Project 7 MPC noise');
end
