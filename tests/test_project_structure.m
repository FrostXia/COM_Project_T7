function test_project_structure
% TEST_PROJECT_STRUCTURE Verify canonical lookup after directory relocation.
root=fileparts(fileparts(mfilename('fullpath')));
addpath(root); setup_paths;
assert(strcmp(TrajectoryAppController.onoff(true),'on') && strcmp(TrajectoryAppController.onoff(false),'off'));
% Registration uses finite numeric IDs and keeps separate runs isolated.
a=simulation_registry('register',[],struct('tag',1));
b=simulation_registry('register',[],struct('tag',2));
assert(isnumeric(a) && isnumeric(b) && a~=b);
first=simulation_registry('get',a); second=simulation_registry('get',b);
assert(first.tag==1 && second.tag==2);
simulation_registry('remove',a);
second=simulation_registry('get',b); assert(second.tag==2);
simulation_registry('remove',b);
expected={ ...
    'TrajectoryAppController', fullfile(root,'Trajectory','src','TrajectoryAppController.m'); ...
    'integrate_robot_motion', fullfile(root,'utils','integrate_robot_motion.m'); ...
    'tracking_error_state', fullfile(root,'utils','tracking_error_state.m'); ...
    'design_feedback_gains', fullfile(root,'controller','design_feedback_gains.m'); ...
    'solve_nonlinear_mpc', fullfile(root,'controller','mpc','solve_nonlinear_mpc.m'); ...
    'mpc_controller_adapter', fullfile(root,'adapter','mpc_controller_adapter.m'); ...
    'external_controller', fullfile(root,'adapter','external_controller.m')};
for j=1:size(expected,1)
    assert(strcmpi(which(expected{j,1}),expected{j,2}), ...
        'Trajectory:Lookup','Unexpected function path for %s.',expected{j,1});
end
assert(norm(wrap_angle([-3*pi -pi 0 pi 3*pi],true)-[-pi -pi 0 pi pi])<1e-12);
c=default_config;
for name={'sf','lqr','mpc'}
    cc=c; cc.controller=name{1}; validate_simulation_config(cc);
end
% One fixed design must use each supplied reference feedforward value.
ref.q=zeros(3,c.N+1); ref.w=zeros(c.N,1);
cc=c; cc.controller='sf';
d=struct('K',[0 -1 -2],'Klqr',[0 -0.5 -1]);
memory=struct('design',d);
[w1,~,i1]=feedback_controller_adapter([0;0;0],ref,cc,memory);
ref.w(:)=0.2;
[w2,~,i2]=feedback_controller_adapter([0;0;0],ref,cc,memory);
assert(i1.ok && i2.ok && abs(w2-w1-0.2)<1e-12);
ref.w(:)=0;
[w3,~,i3]=feedback_controller_adapter([0;0;0.1],ref,cc,memory);
assert(i3.ok && abs(w3+0.2)<1e-12);
disp('PASS: canonical lookup, registry isolation and controller adapters');
end
