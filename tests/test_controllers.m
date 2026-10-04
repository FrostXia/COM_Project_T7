function test_controllers
% TEST_CONTROLLERS Verify shared feedback and nonlinear MPC without experiments.
root=fileparts(fileparts(mfilename('fullpath'))); addpath(root); setup_paths;
assert(norm(tracking_error_state([0;0;0],[1;2;.1])-[1;2;.1])<1e-12);
assert(norm(tracking_error_state([0;0;pi/2],[1;0;pi/2])-[0;-1;0])<1e-12);
if exist('place','file')~=0 && exist('lqr','file')~=0
    [K,Klqr,d]=design_feedback_gains([],false);
    assert(norm(K-[-9.3762584282 -26.7662994499 -7])<1e-7);
    assert(norm(Klqr-[.52660538 -1.31251163 -1.52069446])<1e-7);
    assert(d.careResidual<1e-8);
else
    fprintf('SKIP: Control System Toolbox gain checks.\n');
end
c=default_config; c.N=4;
p=c.algorithm; p.N=c.N; p.Ts=c.Ts; p.v0=c.v0;
p.wr=.2*ones(c.N,1); p.vr=c.v0*ones(c.N,1);
[f,g,E]=mpc_objective(zeros(c.N,1),zeros(3,1),p);
assert(abs(f)<1e-12 && isempty(g) && max(abs(E),[],'all')<1e-12);
% A straight reference has an exact constant-rate body-frame error solution.
p.wr(:)=0; u=.3;
next=predict_tracking_error(zeros(3,1),u,p,1);
q=integrate_robot_motion(zeros(3,1),u,c); qr=[c.v0*c.Ts;0;0];
assert(norm(next-tracking_error_state(q,qr))<1e-8);
p.wr(:)=.2;
[f,~,E]=mpc_objective(.02*ones(c.N,1),[.1;-.05;.03],p);
assert(isfinite(f) && f>0 && isequal(size(E),[3 c.N+1]));
if exist('fmincon','file')~=0
    c.controller='mpc'; c.mpc=c.algorithm;
    [w,~,info]=solve_nonlinear_mpc([.1;-.05;.03],p.wr,p.vr,c,[]);
    assert(info.ok && w>=c.wMin-1e-6 && w<=c.wMax+1e-6);
    params=c.preset; params.circle.radius=3;
    path=generate_trajectory('circle',params,[],c);
    [ref,~]=sample_reference_preview(c.q0,0,path,c,[]);
    [w,~,info]=mpc_controller_adapter(ref.q(:,1),ref,c,[]);
    assert(info.ok && isfinite(w) && isequal(size(info.predictedQ),[3 c.N+1]));
    assert(max(abs(info.predictedQ-ref.q),[],'all')<1e-8);
else
    fprintf('SKIP: Optimization Toolbox solver checks.\n');
end
disp('PASS: body-frame error, feedback gains and nonlinear MPC');
end
