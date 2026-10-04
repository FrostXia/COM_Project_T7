function folder = export_experiment(result,root)
% EXPORT_EXPERIMENT Export one desired trajectory and one evaluation per experiment.
folder=fullfile(root,'results');
if ~isfolder(folder), mkdir(folder); end
stamp=char(datetime('now','Format','yyyyMMdd_HHmmss_SSS'));
base=stamp; suffix=0;
while isfile(fullfile(folder,['trajectory_' stamp '.csv'])) || ...
        isfile(fullfile(folder,['evaluation_' stamp '.csv']))
    suffix=suffix+1; stamp=sprintf('%s_%d',base,suffix);
end
% Headerless XY matches the existing Import Existing Trajectory reader.
writematrix(result.path.xy,fullfile(folder,['trajectory_' stamp '.csv']));
evaluation=evaluate_experiment(result);
evaluation=[table("Export Timestamp",string(stamp),"", ...
    'VariableNames',evaluation.Properties.VariableNames);evaluation];
writetable(evaluation,fullfile(folder,['evaluation_' stamp '.csv']),'Encoding','UTF-8');
end
