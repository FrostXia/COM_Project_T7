function modelFile = build_simulation_model
% BUILD_SIMULATION_MODEL Generate the inspectable normal-mode Simulink harness.
root=fileparts(fileparts(mfilename('fullpath')));
addpath(root);
setup_paths;
assert(~isempty(ver('simulink')),'Trajectory:Dependency','Simulink is required to build this model.');
folder=fullfile(root,'simulation','models'); if ~isfolder(folder), mkdir(folder); end
name='trajectory_simulation'; modelFile=fullfile(folder,[name '.slx']);
if isfile(modelFile), return; end
load_system('simulink');
if bdIsLoaded(name), error('Trajectory:Model','Close the existing unsaved trajectory_simulation first.'); end
new_system(name);
clean=onCleanup(@() close_system(name,0));
set_param(name,'SolverType','Fixed-step','Solver','FixedStepDiscrete', ...
    'FixedStep','0.1','StopTime','10','SimulationMode','normal','ReturnWorkspaceOutputs','on');
add_block('simulink/User-Defined Functions/Level-2 MATLAB S-Function',[name '/Robot plant'], ...
    'FunctionName','sfun_robot_plant','Parameters','0,0.1','Position',[520 125 690 205]);
add_block('simulink/User-Defined Functions/Level-2 MATLAB S-Function',[name '/Reference and controller'], ...
    'FunctionName','sfun_trajectory_controller','Parameters','0,0.1','Position',[140 125 350 205]);
add_block('simulink/Sinks/Scope',[name '/True pose scope'],'Position',[775 135 815 175]);
add_line(name,'Reference and controller/1','Robot plant/1','autorouting','on');
add_line(name,'Robot plant/1','Reference and controller/1','autorouting','on');
add_line(name,'Robot plant/1','True pose scope/1','autorouting','on');
a=Simulink.Annotation(name,sprintf(['Trajectory\n' ...
    'Run from start_app. The app supplies a per-run session.\n' ...
    'The session adds bounded measurement noise before control. The plant keeps true pose.\n' ...
    'Normal simulation only; no code generation, Fast Restart or accelerator mode.']));
a.Position=[130 20];
save_system(name,modelFile);
end
