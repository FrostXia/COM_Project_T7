function c=prepare_controller_design(path,c)
% PREPARE_CONTROLLER_DESIGN Resolve one fixed SF/LQR design for a complete run.
if ~ismember(c.controller,{'sf','lqr'}), return; end
if ~isfield(c,'nominalTurn'), c.nominalTurn=struct('thresholdDeg',1,'fallbackDeg',1); end
a=compute_nominal_turn(path,c.v0,c.nominalTurn);
if ~isfield(c,'design')
    [~,~,d]=design_feedback_gains([],false,true); c.design=d.parameters;
end
c.design.v0=c.v0; c.design.wrNom=a.radiansPerSecond; a.source='automatic';
if isfield(c,'nominalOverride') && ~isempty(c.nominalOverride)
    validateattributes(c.nominalOverride,{'double'},{'scalar','real','finite','nonzero'});
    c.design.wrNom=c.nominalOverride; a.source='manual';
end
a.selectedRadiansPerSecond=c.design.wrNom;
c.nominalReference=a;
end
