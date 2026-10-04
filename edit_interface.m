function edit_interface(view)
% EDIT_INTERFACE Open a native MLAPP in App Designer Design View.
if nargin<1, view='main'; end
root=setup_paths;
switch lower(char(view))
    case 'main', name='TrajectoryDesignerApp.mlapp';
    case {'path','trajectory'}, name='TrajectoryEditorApp.mlapp';
    case 'noise', name='TrajectoryNoiseApp.mlapp';
    otherwise, error('Trajectory:DesignerView','Choose main, path or noise.');
end
appdesigner(fullfile(root,'Trajectory','designer',name));
end
