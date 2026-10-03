function app = start_trajectory
% START_TRAJECTORY Launch the simulation and path editing application.
root = fileparts(mfilename('fullpath'));
addpath(fileparts(fileparts(mfilename('fullpath'))));
setup_project7;
app = P7App(root);
if nargout == 0
    clear app
end
end
