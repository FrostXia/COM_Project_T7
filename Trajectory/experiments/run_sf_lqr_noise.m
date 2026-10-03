function runs=run_sf_lqr_noise(c,inputPath)
% RUN_SF_LQR_NOISE Task 3A: open loop, pole placement and LQR in one Q5 package.
addpath(fileparts(fileparts(fileparts(mfilename('fullpath')))));
setup_project7;
if nargin<1 || isempty(c), c=p7utils.defaults; end
if nargin<2, inputPath=[]; end
runs=p7utils.run_cases(c,inputPath,{'openloop','sf','lqr'},'Project 7 SF and LQR noise');
end
