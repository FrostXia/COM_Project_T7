function runs=run_cases(c,inputPath,controllers,label)
% RUN_CASES One reference, one configuration, independent memories per run.
if nargin<2, inputPath=[]; end
p7utils.validate(c);
reference=p7utils.reference_data(c,inputPath);
design=[];
if any(ismember(controllers,{'sf','lqr'}))
    [~,~,design]=project7_q4_gains(c.design,false);
end
if any(strcmp(controllers,'mpc'))
    assert(exist('fmincon','file')~=0,'P7:Toolbox','MPC requires Optimization Toolbox (fmincon).');
end
runs=cell(1,numel(controllers)*numel(c.scenarios)); n=0;
for j=1:numel(controllers)
    for k=1:numel(c.scenarios)
        n=n+1;
        runs{n}=p7utils.simulate_q5(c,reference,controllers{j},c.scenarios{k},design);
        fprintf('%s / %s: %s, position RMSE %.6g m\n',controllers{j}, ...
            c.scenarios{k},runs{n}.status,runs{n}.metrics.positionRMSE);
    end
end
if c.plot, p7utils.plot_noise(runs,label); end
if c.save, p7utils.export_noise(runs,c,label); end
end
