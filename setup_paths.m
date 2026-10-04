function root = setup_paths
% SETUP_PATHS Add one canonical copy of each project module to the path.
root = fileparts(mfilename('fullpath'));
retired = {fullfile(root,'TrajectoryLab'),fullfile(root,'Trajectory','experiments')};
current = strsplit(path,pathsep);
for k=1:numel(current)
    for j=1:numel(retired)
        if strcmpi(current{k},retired{j}) || startsWith(lower(current{k}),[lower(retired{j}) filesep])
            rmpath(current{k});
            break
        end
    end
end
folders = {root, fullfile(root,'Trajectory'), fullfile(root,'Trajectory','src'), ...
    fullfile(root,'Trajectory','designer'), ...
    fullfile(root,'utils'), ...
    fullfile(root,'controller'), fullfile(root,'controller','mpc'), ...
    fullfile(root,'adapter'), fullfile(root,'tests')};
addpath(folders{:});
end
