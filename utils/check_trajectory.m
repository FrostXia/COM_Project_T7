function report = check_trajectory(path,c)
% CHECK_TRAJECTORY Check sampled geometry, not global closed-loop convergence.
enforceTurn=true;
if isfield(c,'enforcePathTurnLimit'), enforceTurn=c.enforcePathTurnLimit; end
w = c.v0*path.kappa;
turnBad=w<c.wMin-1e-6 | w>c.wMax+1e-6;
bad = ~isfinite(w) | path.tangentNorm<0.2 | (enforceTurn & turnBad);
checkXY=path.xy; checkedW=w; checkedBad=bad;
if isfield(path,'validation')
    checkXY=path.validation.xy;
    checkedW=c.v0*path.validation.kappa;
    checkedTurnBad=checkedW<c.wMin-1e-6 | checkedW>c.wMax+1e-6;
    checkedBad=~isfinite(checkedW) | path.validation.tangentNorm<0.2 | ...
        (enforceTurn & checkedTurnBad);
end
report.ok = ~any(bad) && ~any(checkedBad);
report.bad = bad;
report.badXY = [path.xy(bad,:);checkXY(checkedBad,:)];
report.maxRequiredTurn = max(abs([w;checkedW]));
report.length = path.length;
report.turnFeasible=all(isfinite(checkedW)) && all(checkedW>=c.wMin-1e-6 & checkedW<=c.wMax+1e-6) && ~any(turnBad);
report.warnings = {};
if ~enforceTurn && ~report.turnFeasible
    report.warnings{end+1}=sprintf('Tight turns need %.3f rad/s; controller limits remain [%.3f, %.3f]. Testing is allowed, but tracking may be poor.', ...
        report.maxRequiredTurn,c.wMin,c.wMax);
end
if ~report.ok
    report.summary = sprintf('FAIL: processed path needs peak turn %.3f rad/s; allowed [%.3f, %.3f]. Check red samples.', ...
        report.maxRequiredTurn,c.wMin,c.wMax);
else
    report.summary = sprintf('PASS: sampled geometry. Length %.2f m; peak required turn %.3f rad/s.',path.length,report.maxRequiredTurn);
end
% Detect nonlocal close approaches without rejecting valid crossings.
n = numel(path.s); ids = unique(round(linspace(1,n,min(n,350))));
near = false;
for i=1:numel(ids)
    ds = abs(path.s(ids)-path.s(ids(i)));
    dist = vecnorm(path.xy(ids,:)-path.xy(ids(i),:),2,2);
    if any(ds>max(1,3*c.lookahead) & dist<2*c.pathSpacing)
        near = true; break
    end
end
if near
    report.warnings{end+1} = 'Nonlocal close sections detected; crossing order needs progress management.';
end
if strcmp(path.kind,'freehand')
    if isfield(path,'smoothing')
        sm=path.smoothing;
        report.warnings{end+1}=sprintf('Smoothing shift %.3f m / %.3f m budget; effective span %.2f m.', ...
            sm.maxDeviation,sm.deviationBudget,sm.spanMetres);
        if ~report.ok
            report.warnings{end+1}='No passing candidate found within this shape budget. Increase Max smoothing shift or redraw the tight bend.';
        end
    end
    report.warnings{end+1} = 'Check the processed curve visually. Sampled curvature checks are not a continuous-path proof.';
end
end
