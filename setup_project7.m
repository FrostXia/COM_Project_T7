function root = setup_project7
% SETUP_PROJECT7 Add one canonical copy of each project module to the path.
root = fileparts(mfilename('fullpath'));
old = fullfile(root,'TrajectoryLab');
current = strsplit(path,pathsep);
for k=1:numel(current)
    if strcmpi(current{k},old) || startsWith(lower(current{k}),[lower(old) filesep])
        rmpath(current{k});
    end
end
folders = {root, fullfile(root,'Trajectory'), fullfile(root,'Trajectory','src'), ...
    fullfile(root,'Trajectory','experiments'), fullfile(root,'utils'), ...
    fullfile(root,'controller'), fullfile(root,'controller','mpc'), ...
    fullfile(root,'adapter'), fullfile(root,'tests')};
addpath(folders{:});
end
