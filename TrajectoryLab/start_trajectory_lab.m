function app = start_trajectory_lab
% START_TRAJECTORY_LAB Launch the simulation and path editing application.
root = fileparts(mfilename('fullpath'));
addpath(root, fullfile(root,'src'), fullfile(root,'adapters'), fullfile(root,'tests'));
app = P7App(root);
if nargout == 0
    clear app
end
end
