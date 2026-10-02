function result = run_p7_demo(engine)
% RUN_P7_DEMO Run the demonstration without opening the interactive app.
if nargin<1, engine='simulink'; end
root=fileparts(mfilename('fullpath'));
addpath(root,fullfile(root,'src'),fullfile(root,'adapters'),fullfile(root,'tests'));
c=p7_defaults; path=p7_make_path('circle',[12 3 0.15 1],[],c);
result=p7_run(path,c,[],engine);
disp(result.status); disp(result.metrics);
folder=p7_export(result,root); fprintf('Results: %s\n',folder);
end
