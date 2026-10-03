function test_project7_structure
% TEST_PROJECT7_STRUCTURE Verify canonical lookup after directory relocation.
root=fileparts(fileparts(mfilename('fullpath')));
addpath(root); setup_project7;
expected={ ...
    'P7App', fullfile(root,'Trajectory','src','P7App.m'); ...
    'p7_motion', fullfile(root,'utils','p7_motion.m'); ...
    'p7utils.defaults', fullfile(root,'utils','+p7utils','defaults.m'); ...
    'project7_q4_gains', fullfile(root,'controller','project7_q4_gains.m'); ...
    'p7_mpc_error_step', fullfile(root,'controller','mpc','p7_mpc_error_step.m'); ...
    'project7_mpc_step', fullfile(root,'adapter','project7_mpc_step.m'); ...
    'p7_external_controller', fullfile(root,'adapter','p7_external_controller.m')};
for j=1:size(expected,1)
    assert(strcmpi(which(expected{j,1}),expected{j,2}), ...
        'P7:Lookup','Unexpected function path for %s.',expected{j,1});
end
assert(norm(p7_wrap([-3*pi -pi 0 pi 3*pi],true)-[-pi -pi 0 pi pi])<1e-12);
c=p7_defaults; p=p7_make_path('line',[12 3 .15 1],[],c);
for name={'sf','lqr','mpc'}
    cc=c; cc.controller=name{1}; p7_validate_config(cc);
end
% One fixed design must use each supplied reference feedforward value.
ref.q=zeros(3,c.N+1); ref.w=zeros(c.N,1);
cc=c; cc.controller='sf';
d=struct('K',[0 -1 -2],'Klqr',[0 -0.5 -1]);
memory=struct('design',d);
[w1,~,i1]=p7_controller_adapter([0;0;0],ref,cc,memory);
ref.w(:)=0.2;
[w2,~,i2]=p7_controller_adapter([0;0;0],ref,cc,memory);
assert(i1.ok && i2.ok && abs(w2-w1-0.2)<1e-12);
ref.w(:)=0;
[w3,~,i3]=p7_controller_adapter([0;0;0.1],ref,cc,memory);
assert(i3.ok && abs(w3+0.2)<1e-12);
% Package and platform reference generators share the same path query.
ec=p7utils.defaults; ec.duration=0.2; ec.N=4; ec.plot=false; ec.save=false;
r=p7utils.reference_data(ec,p);
pc=c; pc.N=ec.N; pc.referenceMode='time';
[window,~]=p7_reference([99;99;0],0,p,pc,[]);
assert(norm(window.q-r.q(:,1:ec.N+1),'fro')<1e-12);
disp('PASS: relocated lookup, adapters and shared reference geometry');
end
