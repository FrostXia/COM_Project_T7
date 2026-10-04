function app = start_app
% START_APP Launch the Trajectory GUI from the repository root.
setup_paths;
app = start_trajectory;
if nargout == 0, clear app; end
end
