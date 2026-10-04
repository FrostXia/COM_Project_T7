function report = summarize_trajectory(path,c)
% SUMMARIZE_TRAJECTORY Describe a processed path without testing feasibility.
validateattributes(path.xy,{'double'},{'2d','ncols',2,'real','finite','nonempty'});
report=struct('ok',true,'checked',false,'bad',false(size(path.s)), ...
    'badXY',zeros(0,2),'maxRequiredTurn',max(abs(c.v0*path.kappa)), ...
    'length',path.length,'warnings',{{}},'summary','Processed path ready for confirmation.');
end
