function sfun_robot_plant(block)
% SFUN_ROBOT_PLANT Discrete plant with exact held-input integration; sensor noise is applied in control.
block.NumDialogPrms=2;
% Parameters are the numeric session ID and explicit discrete sample period.
block.DialogPrmsTunable={'Nontunable','Nontunable'};
block.NumInputPorts=1; block.NumOutputPorts=1;
block.SetPreCompInpPortInfoToDynamic; block.SetPreCompOutPortInfoToDynamic;
block.InputPort(1).Dimensions=1; block.InputPort(1).DirectFeedthrough=false;
block.OutputPort(1).Dimensions=3;
% Setup also runs during model construction, before a session exists.
% Explicit sample times avoid any hidden delay or estimator dependency.
validateattributes(block.DialogPrm(2).Data,{'double'},{'scalar','real','finite','positive'});
block.SampleTimes=[block.DialogPrm(2).Data 0];
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
