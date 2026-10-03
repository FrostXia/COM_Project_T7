function path = p7_smooth_drawn(raw,c)
% P7_SMOOTH_DRAWN Remove drawing jitter within an explicit shape-change budget.
% Penalize second differences, preserve endpoints in the solve, and evaluate
% a C2 cubic spline analytically. Never clip curvature to force a pass.
validateattributes(raw,{'double'},{'2d','ncols',2,'finite','real'});
assert(size(raw,1)>=4,'P7:Path','Draw at least four distinct points.');
original=raw;
raw=raw([true;vecnorm(diff(raw),2,2)>1e-6],:);
assert(size(raw,1)>=4,'P7:Path','Draw at least four distinct points.');
u0=[0;cumsum(vecnorm(diff(raw),2,2))];
assert(u0(end)>3*c.pathSpacing,'P7:Path','The stroke is too short.');
assert(u0(end)/c.pathSpacing<100000,'P7:Path','Increase path spacing for this long stroke.');
% Use uniform input samples so mouse speed does not set the smoothing weight.
m=max(20,ceil(u0(end)/c.pathSpacing)+1);
u=linspace(0,u0(end),m)';
points=interp1(u0,raw,u,'linear');
ds=u(2)-u(1);
i=(1:m-2)';
D=sparse([i;i;i],[i;i+1;i+2],[ones(m-2,1);-2*ones(m-2,1);ones(m-2,1)],m-2,m);
penalty=D'*D;
budget=0.20;
if isfield(c,'maxPathDeviation'), budget=c.maxPathDeviation; end
base=max(1,round(c.smoothingWindow));
if base<=1
    spans=0;
else
    % Start gently, then increase smoothing only within the deviation budget.
    spans=[0,unique(min(m,base*1.6.^(0:12)))];
end
% Dense checks include between-knot curvature and displacement, not just nodes.
ud=linspace(0,u(end),4*(m-1)+1)';
originalDense=interp1(u0,raw,ud,'linear');
best=[]; bestScore=inf; bestSpan=0; bestShift=inf; attempts=0;
for span=spans
    attempts=attempts+1;
    lambda=(span/(2*pi))^4;
    A=speye(m)+lambda*penalty;
    candidate=points;
    inside=2:m-1;
    candidate(inside,:)=A(inside,inside)\(points(inside,:)-A(inside,[1 m])*points([1 m],:));
    px=spline(u,candidate(:,1)); py=spline(u,candidate(:,2));
    [xy,~,kap,speed]=evaluate(px,py,ud);
    shift=max(vecnorm(xy-originalDense,2,2));
    if shift>budget+1e-9
        continue
    end
    w=c.v0*kap;
    score=max([max(w/c.wMax),max(w/c.wMin),0.2/max(min(speed),1e-12)]);
    if any(~isfinite(kap)), score=inf; end
    passingSmooth=span>0 && score<=1+1e-7;
    if isempty(best) || score<bestScore || passingSmooth
        best=struct('px',px,'py',py,'xy',xy,'u',ud,'kappa',kap,'speed',speed);
        bestScore=score; bestSpan=span; bestShift=shift;
    end
    if score<=1+1e-7 && (span>0 || base<=1)
        % Choose the smallest successful smoothing level, not the straightest path.
        break
    end
end
if isempty(best)
    error('P7:ShapeBudget', ...
        'No smooth candidate stays within %.3f m of the stroke. Increase Max smoothing shift or redraw.',budget);
end
sd=[0;cumsum(vecnorm(diff(best.xy),2,2))];
assert(sd(end)>3*c.pathSpacing,'P7:Path','Smoothed path is too short.');
keep=[true;diff(sd)>1e-10];
s=linspace(0,sd(end),max(20,ceil(sd(end)/c.pathSpacing)+1))';
uq=interp1(sd(keep),best.u(keep),s,'linear');
[xy,theta,kap,speed]=evaluate(best.px,best.py,uq);
% The local derivative is with respect to the original arc-length parameter.
% Its magnitude detects reversals and vanishing tangents during smoothing.
path=struct('kind','freehand','raw',original,'s',s,'xy',xy, ...
    'theta',theta,'kappa',kap,'tangentNorm',speed,'length',s(end));
path.validation=struct('xy',best.xy,'kappa',best.kappa,'tangentNorm',best.speed);
path.smoothing=struct('spanSamples',bestSpan,'spanMetres',bestSpan*ds, ...
    'maxDeviation',bestShift,'deviationBudget',budget,'attempts',attempts, ...
    'autoStrengthened',bestSpan>base,'withinBudget',bestShift<=budget+1e-9);
end

function [xy,theta,kappa,speed]=evaluate(px,py,u)
dx=ppval(derivative(px),u); dy=ppval(derivative(py),u);
ddx=ppval(derivative(derivative(px)),u);
ddy=ppval(derivative(derivative(py)),u);
xy=[reshape(ppval(px,u),[],1),reshape(ppval(py,u),[],1)];
dx=dx(:); dy=dy(:); ddx=ddx(:); ddy=ddy(:);
speed=hypot(dx,dy);
kappa=(dx.*ddy-dy.*ddx)./max(speed.^3,1e-12);
theta=unwrap(atan2(dy,dx));
end

function dp=derivative(p)
% Differentiate the piecewise polynomial without a spline toolbox dependency.
dp=mkpp(p.breaks,p.coefs(:,1:end-1).*(p.order-1:-1:1),p.dim);
end
