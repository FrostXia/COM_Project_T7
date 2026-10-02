function modelFile = build_p7_model
% BUILD_P7_MODEL Generate the inspectable normal-mode Simulink harness.
root=fileparts(mfilename('fullpath'));
addpath(root,fullfile(root,'src'),fullfile(root,'adapters'));
assert(~isempty(ver('simulink')),'P7:Dependency','Simulink is required to build this model.');
folder=fullfile(root,'models'); if ~isfolder(folder), mkdir(folder); end
name='p7_trajectory_harness'; modelFile=fullfile(folder,[name '.slx']);
if isfile(modelFile), return; end
load_system('simulink');
if bdIsLoaded(name), error('P7:Model','Close the existing unsaved p7_trajectory_harness first.'); end
new_system(name);
clean=onCleanup(@() close_system(name,0));
set_param(name,'SolverType','Fixed-step','Solver','FixedStepDiscrete', ...
    'FixedStep','0.1','StopTime','10','SimulationMode','normal','ReturnWorkspaceOutputs','on');
add_block('simulink/User-Defined Functions/Level-2 MATLAB S-Function',[name '/Robot plant'], ...
    'FunctionName','p7_sf_plant','Parameters','p7SessionKey','Position',[520 125 690 205]);
add_block('simulink/User-Defined Functions/Level-2 MATLAB S-Function',[name '/Sensor and estimator'], ...
    'FunctionName','p7_sf_observer','Parameters','p7SessionKey','Position',[150 315 350 395]);
add_block('simulink/User-Defined Functions/Level-2 MATLAB S-Function',[name '/Reference and controller'], ...
    'FunctionName','p7_sf_controller','Parameters','p7SessionKey','Position',[140 125 350 205]);
add_block('simulink/Discrete/Unit Delay',[name '/Previous applied omega'], ...
    'SampleTime','p7Ts','InitialCondition','0','Position',[435 330 475 370]);
add_block('simulink/Sinks/Terminator',[name '/Measurement monitor'], ...
    'Position',[395 420 415 440]);
add_block('simulink/Sinks/Scope',[name '/True pose scope'],'Position',[775 135 815 175]);
add_line(name,'Reference and controller/1','Robot plant/1','autorouting','on');
add_line(name,'Reference and controller/1','Previous applied omega/1','autorouting','on');
add_line(name,'Robot plant/1','Sensor and estimator/1','autorouting','on');
add_line(name,'Robot plant/1','True pose scope/1','autorouting','on');
add_line(name,'Previous applied omega/1','Sensor and estimator/2','autorouting','on');
add_line(name,'Sensor and estimator/1','Reference and controller/1','autorouting','on');
add_line(name,'Sensor and estimator/2','Measurement monitor/1','autorouting','on');
a=Simulink.Annotation(name,sprintf(['Project 7 | TrajectoryLab\n' ...
    'Run from start_trajectory_lab or run_p7_demo. The app supplies a per-run session.\n' ...
    'The controller receives estimated pose only. Plant integration is independent.\n' ...
    'Normal simulation only; no code generation, Fast Restart or accelerator mode.']));
a.Position=[130 20];
save_system(name,modelFile);
end
