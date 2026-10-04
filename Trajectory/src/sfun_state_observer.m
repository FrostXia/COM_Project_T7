function sfun_state_observer(block)
% SFUN_STATE_OBSERVER Sensor boundary and state estimator, isolated from control.
block.NumDialogPrms=1;
% This numeric ID identifies a run; it is not an online tuning parameter.
block.DialogPrmsTunable={'Nontunable'};
block.NumInputPorts=2; block.NumOutputPorts=2;
block.SetPreCompInpPortInfoToDynamic; block.SetPreCompOutPortInfoToDynamic;
block.InputPort(1).Dimensions=3; block.InputPort(2).Dimensions=1;
block.InputPort(1).DirectFeedthrough=true; block.InputPort(2).DirectFeedthrough=true;
block.OutputPort(1).Dimensions=3; block.OutputPort(2).Dimensions=3;
% Setup also runs during model construction, before a session exists.
% Inherit the discrete rate from the explicitly sampled delay in the harness.
block.SampleTimes=[-1 0]; block.SimStateCompliance='DisallowSimState';
block.RegBlockMethod('Outputs',@output);
end
function output(block)
s=simulation_registry('get',block.DialogPrm(1).Data);
[hat,y]=s.observe(block.CurrentTime,block.InputPort(1).Data,block.InputPort(2).Data);
block.OutputPort(1).Data=hat; block.OutputPort(2).Data=y;
end
