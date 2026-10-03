function r=reference_data(c,inputPath)
% REFERENCE_DATA Build one noise-free time reference, once before simulation.
if nargin<2, inputPath=[]; end
p7utils.validate(c);
M=floor(c.duration/c.Ts); path=[]; tab=[];
%% 1. Load a path struct, TrajectoryLab result, XY CSV/DAT, or time table
if ~isempty(inputPath)
    if isstruct(inputPath)
        if isfield(inputPath,'path'), path=inputPath.path; else, path=inputPath; end
    else
        file=char(inputPath); [~,~,ext]=fileparts(file);
        if strcmpi(ext,'.mat')
            a=load(file);
            if isfield(a,'path')
                path=a.path;
            elseif isfield(a,'result') && isfield(a.result,'path')
                path=a.result.path;
            else
                error('P7:PathFile','MAT must contain path or result.path.');
            end
        elseif ismember(lower(ext),{'.csv','.dat','.txt'})
            tab=readmatrix(file,'FileType','text');
            assert(isreal(tab) && all(isfinite(tab(:))),'P7:PathFile','Use finite numeric rows without text headers.');
            if size(tab,2)==2
                g=c; g.v0=c.vr; path=p7_smooth_drawn(tab,g); tab=[];
            else
                assert(size(tab,2)==5,'P7:PathFile', ...
                    'Use XY (2 columns) or [t,xr,yr,theta_r,omega_r] (5 columns).');
                assert(size(tab,1)>=3 && all(diff(tab(:,1))>0) && abs(tab(1,1))<1e-10, ...
                    'P7:PathTime','Time must start at zero and increase strictly.');
                tab(:,4)=unwrap(tab(:,4));
            end
        else
            error('P7:PathFile','Unsupported path file extension.');
        end
    end
elseif ~strcmp(c.referenceKind,'q4circle')
    g=c; g.v0=c.vr;
    path=p7_make_path(c.referenceKind,c.pathParams,[],g);
end
%% 2. Finite path ends the experiment; preview extends on the terminal tangent
if ~isempty(path)
    required={'s','xy','theta','kappa','length'};
    assert(all(isfield(path,required)),'P7:Path','Incomplete TrajectoryLab path.');
    path.s=path.s(:); path.theta=path.theta(:); path.kappa=path.kappa(:);
    n=numel(path.s);
    assert(n>=3 && isequal(size(path.xy),[n 2]) && numel(path.theta)==n && numel(path.kappa)==n, ...
        'P7:Path','Path dimensions are inconsistent.');
    vals=[path.s;path.xy(:);path.theta;path.kappa;path.length];
    assert(all(isfinite(vals)) && all(diff(path.s)>0) && abs(path.s(1))<1e-10 && ...
        abs(path.s(end)-path.length)<1e-6*max(1,path.length),'P7:Path','Invalid arc-length table.');
    path.theta=unwrap(path.theta);
    M=min(M,floor(path.length/c.vr/c.Ts));
elseif ~isempty(tab)
    M=min(M,floor(tab(end,1)/c.Ts));
end
assert(M>=1,'P7:Reference','Reference must last at least one sampling interval.');
t=(0:M+c.N)*c.Ts;
q=zeros(3,numel(t)); w=zeros(1,numel(t)); vr=c.vr*ones(1,numel(t));
%% 3. Generate geometry or exactly match the Q5 reference integration
if ~isempty(path)
    g=c; g.v0=c.vr;
    [q,ww]=p7_path_query(path,c.vr*t,g); w=ww.';
elseif ~isempty(tab)
    tc=min(t,tab(end,1));
    q=interp1(tab(:,1),tab(:,2:4),tc,'linear').';
    w=interp1(tab(:,1),tab(:,5),tc,'linear');
    tail=t>tab(end,1);
    q(1,tail)=q(1,tail)+c.vr*(t(tail)-tab(end,1))*cos(tab(end,4));
    q(2,tail)=q(2,tail)+c.vr*(t(tail)-tab(end,1))*sin(tab(end,4));
    w(tail)=0;
else
    q(:,1)=c.qr0; w(:)=c.design.wrNom;
    for k=1:numel(t)-1
        q(:,k+1)=p7utils.pose_step(q(:,k),c.vr,w(k),c.Ts,c.integrationSteps);
    end
end
%% 4. Report reference/model consistency instead of assuming exact consistency
res=zeros(3,numel(t)-1);
for k=1:numel(t)-1
    qp=p7utils.pose_step(q(:,k),vr(k),w(k),c.Ts,c.integrationSteps);
    res(:,k)=p7utils.error_state(qp,q(:,k+1));
end
r=struct('t',t,'q',q,'w',w,'vr',vr,'steps',M,'path',path, ...
    'maxModelResidual',max(abs(res),[],2),'source',c.referenceKind);
if ~isempty(inputPath), r.source='provided_path'; end
end
