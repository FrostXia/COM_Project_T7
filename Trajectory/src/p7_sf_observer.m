function p7_sf_observer(block)
% P7_SF_OBSERVER Sensor boundary and state estimator, isolated from control.
block.NumDialogPrms=1;
s=p7_registry('get',block.DialogPrm(1).Data);
block.NumInputPorts=2; block.NumOutputPorts=2;
block.SetPreCompInpPortInfoToDynamic; block.SetPreCompOutPortInfoToDynamic;
block.InputPort(1).Dimensions=3; block.InputPort(2).Dimensions=1;
block.InputPort(1).DirectFeedthrough=true; block.InputPort(2).DirectFeedthrough=true;
block.OutputPort(1).Dimensions=3; block.OutputPort(2).Dimensions=3;
block.SampleTimes=[s.Config.Ts 0]; block.SimStateCompliance='DisallowSimState';
block.RegBlockMethod('Outputs',@output);
end
function output(block)
s=p7_registry('get',block.DialogPrm(1).Data);
[hat,y]=s.observe(block.CurrentTime,block.InputPort(1).Data,block.InputPort(2).Data);
block.OutputPort(1).Data=hat; block.OutputPort(2).Data=y;
end
