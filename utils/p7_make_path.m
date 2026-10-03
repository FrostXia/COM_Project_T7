function path = p7_make_path(kind,params,raw,c)
% P7_MAKE_PATH Convert analytic or drawn paths to a shared arc-length table.
% params is [length, radius, spiralGrowth, turns]. All lengths are in metres.
if nargin<3, raw=[]; end
switch lower(kind)
    case 'line'
        assert(params(1)>0,'P7:Path','Line length must be positive.');
        xy = [linspace(0,params(1),max(100,ceil(params(1)/c.pathSpacing)+1))',zeros(max(100,ceil(params(1)/c.pathSpacing)+1),1)];
    case 'circle'
        assert(params(2)>0 && params(4)>0,'P7:Path','Radius and turns must be positive.');
        a = linspace(0,2*pi*params(4),max(400,ceil(2*pi*params(2)*params(4)/c.pathSpacing)+1))';
        xy = [params(2)*sin(a),params(2)*(1-cos(a))];
    case 'spiral'
        assert(params(2)>0 && params(3)>=0 && params(4)>0,'P7:Path','Invalid spiral parameters.');
        a = linspace(0,2*pi*params(4),max(800,ceil(2*pi*params(4)*(params(2)+params(3)*2*pi*params(4))/c.pathSpacing)+1))';
        r = params(2)+params(3)*a;
        xy = [r.*cos(a)-params(2),r.*sin(a)];
    case 'freehand'
        path = p7_smooth_drawn(raw,c);
        return
    otherwise
        error('P7:Path','Unknown path type.');
end
original = xy;
xy = xy([true;vecnorm(diff(xy),2,2)>1e-6],:);
assert(size(xy,1)>=4,'P7:Path','Path is too short or contains repeated points.');
s0 = [0;cumsum(vecnorm(diff(xy),2,2))];
assert(s0(end)>3*c.pathSpacing,'P7:Path','Path is too short for the selected spacing.');
assert(s0(end)/c.pathSpacing<100000,'P7:Path','Path has too many samples; increase spacing.');
su = linspace(0,s0(end),max(20,ceil(s0(end)/c.pathSpacing)+1))';
xy = interp1(s0,xy,su,'pchip');
xy = xy([true;vecnorm(diff(xy),2,2)>1e-8],:);
s0 = [0;cumsum(vecnorm(diff(xy),2,2))];
s = linspace(0,s0(end),max(20,ceil(s0(end)/c.pathSpacing)+1))';
xy = interp1(s0,xy,s,'pchip');
dx = gradient(xy(:,1),s); dy = gradient(xy(:,2),s);
ddx = gradient(dx,s); ddy = gradient(dy,s);
speed = hypot(dx,dy);
kappa = (dx.*ddy-dy.*ddx)./max(speed.^3,1e-12);
theta = unwrap(atan2(dy,dx));
if strcmpi(kind,'line')
    theta(:)=0; kappa(:)=0;
elseif strcmpi(kind,'circle')
    theta = linspace(0,2*pi*params(4),numel(s))'; kappa(:)=1/params(2);
end
path = struct('kind',lower(kind),'raw',original,'s',s,'xy',xy, ...
    'theta',theta,'kappa',kappa,'tangentNorm',speed,'length',s(end));
end
