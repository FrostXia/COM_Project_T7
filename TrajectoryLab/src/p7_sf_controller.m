function p7_sf_controller(block)
% P7_SF_CONTROLLER Sampled controller adapter; only estimated pose enters.
block.NumDialogPrms=1;
s=p7_registry('get',block.DialogPrm(1).Data);
block.NumInputPorts=1; block.NumOutputPorts=1;
block.SetPreCompInpPortInfoToDynamic; block.SetPreCompOutPortInfoToDynamic;
block.InputPort(1).Dimensions=3; block.InputPort(1).DirectFeedthrough=true;
block.OutputPort(1).Dimensions=1;
block.SampleTimes=[s.Config.Ts 0]; block.SimStateCompliance='DisallowSimState';
block.RegBlockMethod('Outputs',@output);
end
function output(block)
s=p7_registry('get',block.DialogPrm(1).Data);
block.OutputPort(1).Data=s.control(block.CurrentTime,block.InputPort(1).Data);
if s.StopRequested
    set_param(bdroot(block.BlockHandle),'SimulationCommand','stop');
end
end
