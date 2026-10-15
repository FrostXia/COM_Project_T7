function app = start_app
% START_APP Launch the native interface from this project copy.
setup_paths;
view=TrajectoryDesignerApp;
app=view.Controller;
if nargout==0, clear app; end
end
