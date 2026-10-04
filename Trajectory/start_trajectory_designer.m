function app=start_trajectory_designer
% START_TRAJECTORY_DESIGNER Launch the visually editable App Designer app.
root=fileparts(fileparts(mfilename('fullpath')));
addpath(root); setup_paths;
addpath(fullfile(root,'Trajectory','designer'));
app=TrajectoryDesignerApp;
if nargout==0, clear app; end
end
