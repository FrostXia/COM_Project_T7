function summary=run_batch_trials(count,engine)
% RUN_BATCH_TRIALS Preserve all outcomes, including failed randomized trials.
if nargin<1, count=10; end
if nargin<2, engine='simulink'; end
validateattributes(count,{'double'},{'scalar','integer','positive','<=',1000});
root=fileparts(mfilename('fullpath'));
addpath(fileparts(fileparts(mfilename('fullpath'))));
setup_project7;
base=p7_defaults; path=p7_make_path('line',[12 3 .15 1],[],base);
rows=cell(count,7);
for i=1:count
    c=base; c.seed=base.seed+i;
    stream=RandStream('mt19937ar','Seed',c.seed+10000);
    c.q0=[-2+4*rand(stream);-3+6*rand(stream);-pi+2*pi*rand(stream)];
    r=p7_run(path,c,[],engine); folder=p7_export(r,root);
    rows(i,:)={i,c.seed,r.status,r.metrics.trackingSuccess,r.metrics.positionRMSE,r.metrics.maxSolveTime,folder};
end
summary=cell2table(rows,'VariableNames',{'trial','seed','status','trackingSuccess','positionRMSE','maxSolveTime','folder'});
out=fullfile(root,'results',['batch_' char(datetime('now','Format','yyyyMMdd_HHmmss_SSS')) '.csv']);
writetable(summary,out); disp(summary);
end
