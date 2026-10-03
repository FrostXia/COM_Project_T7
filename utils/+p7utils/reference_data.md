# reference_data.m 逐段说明

**作用：**将预设、手绘结构体或外部文件转为同一无噪声时间参考及预测窗口。

**调用关系：**调用 vendor 路径函数、pose_step、error_state；结果由所有控制器共享。

**边界：**噪声分析始终使用 time 参考，避免反馈相关 path 进度改变 Q5 实验含义。五列表默认速度为 c.vr。

代码中的 %% 是主要逻辑分段；下面按执行顺序解释。短辅助函数没有强行拆成多段。完整源码附在末尾，便于逐项核对。

### 第 1 段：实现与原理

只在仿真开始前读文件。MAT 接受 path 或 TrajectoryLab 导出的 result.path；文本接受两列 XY 或五列时间参考。

### 第 2 段：实现与原理

检查弧长单调、维度和有限值。有限路径限制仿真时长，预测末端沿切线延伸，不假设机器人能停车。

### 第 3 段：实现与原理

默认 q4circle 用与 Q5 实际机器人相同的小步积分推进参考；路径结构体调用原平台查询函数。

### 第 4 段：实现与原理

显式计算每步参考与 Q5 离散运动模型之间的残差，保存到结果；插值参考不被假称为严格离散一致。

## 验证状态

MATLAB 语法树解析通过；相关数学使用 Python/SciPy 作了独立核对。没有在交付环境执行 MATLAB 或 Simulink，具体运行检查见包内 VALIDATION.md。对原平台复制函数，本次只做源码、调用和语法核对，未将原作者的静态记录当作本次运行结果。

## 对应源码

~~~matlab
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
            if isfield(a,'path'), path=a.path;
            elseif isfield(a,'result') && isfield(a.result,'path'), path=a.result.path;
            else, error('P7:PathFile','MAT must contain path or result.path.'); end
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
        else, error('P7:PathFile','Unsupported path file extension.'); end
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
~~~
