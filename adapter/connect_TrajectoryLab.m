function app = connect_TrajectoryLab(projectRoot)
% CONNECT_TRAJECTORYLAB Compatibility connector for the reorganized repository.
if nargin<1 || isempty(projectRoot)
    projectRoot=fileparts(fileparts(mfilename('fullpath')));
end
if ~isfile(fullfile(projectRoot,'setup_paths.m'))
    parent=fileparts(projectRoot);
    assert(isfile(fullfile(parent,'setup_paths.m')),'Trajectory:Platform', ...
        'Select the repository root or its Trajectory directory.');
    projectRoot=parent;
end
addpath(projectRoot); setup_paths;
assert(exist('fmincon','file')~=0,'Trajectory:Toolbox','Optimization Toolbox is required.');
app=start_trajectory;
app.UI.controller.Value='mpc';
app.controllerChanged();
fprintf('MPC connected. Existing estimator and noise selections are retained.\n');
end
