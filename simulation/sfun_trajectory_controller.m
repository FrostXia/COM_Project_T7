function sfun_trajectory_controller(block)
% SFUN_TRAJECTORY_CONTROLLER Sampled controller adapter; receives plant pose; the session adds bounded measurement noise before feedback.
block.NumDialogPrms=2;
% Parameters are the numeric session ID and explicit discrete sample period.
block.DialogPrmsTunable={'Nontunable','Nontunable'};
block.NumInputPorts=1; block.NumOutputPorts=1;
block.SetPreCompInpPortInfoToDynamic; block.SetPreCompOutPortInfoToDynamic;
block.InputPort(1).Dimensions=3; block.InputPort(1).DirectFeedthrough=true;
block.OutputPort(1).Dimensions=1;
% Setup also runs during model construction, before a session exists.
% Explicit sample times avoid any hidden delay or estimator dependency.
validateattributes(block.DialogPrm(2).Data,{'double'},{'scalar','real','finite','positive'});
block.SampleTimes=[block.DialogPrm(2).Data 0]; block.SimStateCompliance='DisallowSimState';
block.RegBlockMethod('Outputs',@output);
end
function output(block)
s=simulation_registry('get',block.DialogPrm(1).Data);
block.OutputPort(1).Data=s.control(block.CurrentTime,block.InputPort(1).Data);
if s.StopRequested
    set_param(bdroot(block.BlockHandle),'SimulationCommand','stop');
end
end
