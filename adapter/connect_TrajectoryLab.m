function app = connect_TrajectoryLab(projectRoot)
% CONNECT_TRAJECTORYLAB Compatibility connector for the reorganized repository.
if nargin<1 || isempty(projectRoot)
    projectRoot=fileparts(fileparts(mfilename('fullpath')));
end
if ~isfile(fullfile(projectRoot,'setup_project7.m'))
    parent=fileparts(projectRoot);
    assert(isfile(fullfile(parent,'setup_project7.m')),'P7:Platform', ...
        'Select the repository root or its Trajectory directory.');
    projectRoot=parent;
end
addpath(projectRoot); setup_project7;
assert(exist('fmincon','file')~=0,'P7:Toolbox','Optimization Toolbox is required.');
app=start_trajectory;
app.UI.controller.Value='mpc';
app.UI.reference.Value='time';
app.UI.mpcMode.Value='nonlinear';
app.invalidate();
fprintf('MPC connected. Existing estimator and noise selections are retained.\n');
end
