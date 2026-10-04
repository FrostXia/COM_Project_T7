function sfun_trajectory_controller(block)
% SFUN_TRAJECTORY_CONTROLLER Sampled controller adapter; only estimated pose enters.
block.NumDialogPrms=1;
% This numeric ID identifies a run; it is not an online tuning parameter.
block.DialogPrmsTunable={'Nontunable'};
block.NumInputPorts=1; block.NumOutputPorts=1;
block.SetPreCompInpPortInfoToDynamic; block.SetPreCompOutPortInfoToDynamic;
block.InputPort(1).Dimensions=3; block.InputPort(1).DirectFeedthrough=true;
block.OutputPort(1).Dimensions=1;
% Setup also runs during model construction, before a session exists.
% Inherit the discrete rate from the explicitly sampled delay in the harness.
block.SampleTimes=[-1 0]; block.SimStateCompliance='DisallowSimState';
block.RegBlockMethod('Outputs',@output);
end
function output(block)
s=simulation_registry('get',block.DialogPrm(1).Data);
block.OutputPort(1).Data=s.control(block.CurrentTime,block.InputPort(1).Data);
if s.StopRequested
    set_param(bdroot(block.BlockHandle),'SimulationCommand','stop');
end
end
