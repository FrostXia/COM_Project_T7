function run_project7_tests
% RUN_PROJECT7_TESTS Execute these checks in MATLAB; not executed at delivery.
root=fileparts(fileparts(mfilename('fullpath')));
addpath(root); setup_project7;
hasMPC=isfolder(fullfile(root,'controller','mpc'));
c=p7utils.defaults; c.duration=0.5; c.N=4; c.plot=false; c.save=false;
%% 0. Blank KF covariance is deliberate, and enabling an unconfigured KF fails
assert(isempty(c.filter.Qn) && isempty(c.filter.Rn));
p7utils.validate(c);
[y,m]=p7utils.kf_error([1;2;0.1],[],c,[]);
assert(isequal(y,[1;2;0.1]) && isempty(m));
cc=c; cc.filter.mode='kf'; rejected=false;
try
    p7utils.validate(cc);
catch ME
    rejected=strcmp(ME.identifier,'P7:KalmanCovarianceUnset');
end
assert(rejected,'KF must reject blank covariances.');
%% 1. Error sign and exact Q5 nominal reference invariance
assert(norm(p7utils.error_state([0;0;0],[1;2;0.1])-[1;2;0.1])<1e-12);
r=p7utils.reference_data(c,[]);
clean=p7utils.simulate_q5(c,r,'openloop','clean',[]);
assert(max(abs(clean.log.errorTrue(:)))<1e-10);
%% 2. Measurement-only noise must not move an open-loop physical trajectory
obs=p7utils.simulate_q5(c,r,'openloop','observer',[]);
assert(isequal(clean.log.qTrue,obs.log.qTrue));
assert(norm(obs.log.errorMeasured(:,2:end),'fro')>0);
ctrl=p7utils.simulate_q5(c,r,'openloop','control',[]);
assert(norm(ctrl.log.qTrue-clean.log.qTrue,'fro')>0);
repeat=p7utils.simulate_q5(c,r,'openloop','control',[]);
assert(isequal(ctrl.log.qTrue,repeat.log.qTrue));
assert(all(abs(ctrl.log.omegaPhysical-ctrl.log.omegaCommand-ctrl.log.controlNoise)<1e-12));
%% 3. Task 1 gains and shared noise sequences
if exist('place','file')~=0 && exist('lqr','file')~=0
    [K,Klqr,d]=project7_q4_gains([],false);
    assert(norm(K-[-9.3762584282 -26.7662994499 -7])<1e-7);
    assert(norm(Klqr-[0.52660538 -1.31251163 -1.52069446])<1e-7);
    a=p7utils.simulate_q5(c,r,'sf','both',d);
    b=p7utils.simulate_q5(c,r,'lqr','both',d);
    assert(isequal(a.log.measurementNoise,b.log.measurementNoise));
    assert(isequal(a.log.controlNoise,b.log.controlNoise));
else
    fprintf('SKIP: Control System Toolbox gain checks.\n');
end
%% 4. Path interface, horizon dimensions and finite endpoint continuation
path=p7_make_path('circle',[12 3 0.15 1],[],c);
rr=p7utils.reference_data(c,path);
assert(size(rr.q,1)==3 && size(rr.q,2)>=rr.steps+c.N);
[qp,wp]=p7_path_query(path,[path.length path.length+1],c);
assert(norm(qp(1:2,2)-qp(1:2,1))>0.9 && wp(2)==0);
%% 5. MPC gradient, finite bounds and platform output contract
if hasMPC
    assert(norm(wrap([-3*pi -pi 0 pi 3*pi])-[-pi -pi 0 pi pi])<1e-12);
    [th,xx,yy]=compute_next_pose_Project7(3.13,1,2,c.v0,0.8,c.Ts);
    truth=p7utils.pose_step([1;2;3.13],c.v0,0.8,c.Ts,100);
    assert(norm([xx;yy]-truth(1:2))<1e-12 && abs(wrap(th-truth(3)))<1e-12);
    p=c.mpc; p.N=c.N; p.Ts=c.Ts; p.v0=c.v0;
    p.wr=(pi/4)*ones(c.N,1); p.vr=c.vr*ones(c.N,1);
    for j=1:c.N
        [p.Ad(:,:,j),p.Bd(:,j),p.dd(:,j)]=p7utils.linear_step(p.wr(j),p.v0,p.vr(j),p.Ts);
    end
    e=[0.1;-0.05;0.03]; U=0.02*ones(c.N,1);
    [~,g]=objfun_Project7_MPC(U,e,p); gn=zeros(c.N,1); h=1e-6;
    for j=1:c.N
        z=zeros(c.N,1); z(j)=h;
        gn(j)=(objfun_Project7_MPC(U+z,e,p)-objfun_Project7_MPC(U-z,e,p))/(2*h);
    end
    assert(norm(g-gn)<1e-6);
    if exist('fmincon','file')~=0
        [w,~,info]=p7_mpc_error_step(e,p.wr,p.vr,c,[]);
        assert(info.ok && w>=c.wMin-1e-6 && w<=c.wMax+1e-6);
        ref=struct('q',r.q(:,1:c.N+1),'w',r.w(1:c.N)');
        [w,~,info]=project7_mpc_step(c.q0,ref,c,[]);
        assert(info.ok && isfinite(w) && isequal(size(info.predictedQ),[3 c.N+1]));
        demo=launch_Project7_MPC(c);
        assert(strcmp(demo.status,'completed') && demo.completed==r.steps);
        assert(max(abs(demo.error(:)))<1e-7);
    else
        fprintf('SKIP: Optimization Toolbox solver checks.\n');
    end
end
fprintf('Completed all available MATLAB checks. Review any SKIP lines above.\n');
end
