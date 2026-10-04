function sfun_robot_plant(block)
% SFUN_ROBOT_PLANT Discrete plant with exact held-input integration and state noise.
block.NumDialogPrms=1;
% This numeric ID identifies a run; it is not an online tuning parameter.
block.DialogPrmsTunable={'Nontunable'};
block.NumInputPorts=1; block.NumOutputPorts=1;
block.SetPreCompInpPortInfoToDynamic; block.SetPreCompOutPortInfoToDynamic;
block.InputPort(1).Dimensions=1; block.InputPort(1).DirectFeedthrough=false;
block.OutputPort(1).Dimensions=3;
% Setup also runs during model construction, before a session exists.
% Inherit the discrete rate from the explicitly sampled delay in the harness.
block.SampleTimes=[-1 0];
block.SimStateCompliance='DisallowSimState';
block.RegBlockMethod('PostPropagationSetup',@post);
block.RegBlockMethod('InitializeConditions',@init);
block.RegBlockMethod('Outputs',@output);
block.RegBlockMethod('Update',@update);
end
function post(block)
block.NumDworks=1; block.Dwork(1).Name='TruePose';
block.Dwork(1).Dimensions=3; block.Dwork(1).DatatypeID=0;
block.Dwork(1).Complexity='Real'; block.Dwork(1).UsedAsDiscState=true;
end
function init(block)
s=simulation_registry('get',block.DialogPrm(1).Data);
block.Dwork(1).Data=s.Config.q0;
end
function output(block)
block.OutputPort(1).Data=block.Dwork(1).Data;
end
function update(block)
s=simulation_registry('get',block.DialogPrm(1).Data);
if ~s.StopRequested
    block.Dwork(1).Data=s.advance(block.Dwork(1).Data,block.InputPort(1).Data);
end
end
