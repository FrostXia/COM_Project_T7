function app = start_trajectory_lab
% START_TRAJECTORY_LAB Compatibility entry for the renamed Trajectory app.
addpath(fileparts(fileparts(mfilename('fullpath'))));
setup_paths;
app = start_trajectory;
if nargout == 0, clear app; end
end
