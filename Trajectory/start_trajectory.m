function app = start_trajectory
% START_TRAJECTORY Launch native App Designer views and return their controller.
addpath(fileparts(fileparts(mfilename('fullpath'))));
setup_paths;
designer = start_trajectory_designer;
app = designer.Controller;
if nargout == 0
    clear app
end
end
