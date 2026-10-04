function path = generate_trajectory(kind,params,raw,c)
% GENERATE_TRAJECTORY Convert analytic or drawn paths to a shared arc-length table.
% params is the named c.preset structure; circle presets always use one turn.
% Legacy numeric vectors remain accepted for independent experiment callers.
if nargin<3, raw=[]; end
circleTurns=1;
if ~strcmpi(kind,'freehand') && isnumeric(params)
    validateattributes(params,{'double'},{'vector','numel',4,'finite','real'});
    old=params;
    params=struct('line',struct('length',old(1)), ...
        'circle',struct('radius',old(2)), ...
        'spiral',struct('initialRadius',old(2),'growthRate',old(3),'turns',old(4)));
    circleTurns=old(4);
end
switch lower(kind)
    case 'line'
        length=params.line.length;
        validateattributes(length,{'double'},{'scalar','finite','real','positive'});
        count=max(100,ceil(length/c.pathSpacing)+1);
        xy = [linspace(0,length,count)',zeros(count,1)];
    case 'circle'
        radius=params.circle.radius;
        validateattributes(radius,{'double'},{'scalar','finite','real','positive'});
        validateattributes(circleTurns,{'double'},{'scalar','finite','real','positive'});
        a = linspace(0,2*pi*circleTurns,max(400,ceil(2*pi*radius*circleTurns/c.pathSpacing)+1))';
        xy = [radius*sin(a),radius*(1-cos(a))];
    case 'spiral'
        radius=params.spiral.initialRadius;
        growth=params.spiral.growthRate; turns=params.spiral.turns;
        validateattributes(radius,{'double'},{'scalar','finite','real','positive'});
        validateattributes(growth,{'double'},{'scalar','finite','real','nonnegative'});
        validateattributes(turns,{'double'},{'scalar','finite','real','positive'});
        a = linspace(0,2*pi*turns,max(800,ceil(2*pi*turns*(radius+growth*2*pi*turns)/c.pathSpacing)+1))';
        r = radius+growth*a;
        xy = [r.*cos(a)-radius,r.*sin(a)];
    case 'freehand'
        path = smooth_drawn_trajectory(raw,c);
        return
    otherwise
        error('Trajectory:Path','Unknown path type.');
end
original = xy;
xy = xy([true;vecnorm(diff(xy),2,2)>1e-6],:);
assert(size(xy,1)>=4,'Trajectory:Path','Path is too short or contains repeated points.');
s0 = [0;cumsum(vecnorm(diff(xy),2,2))];
assert(s0(end)>3*c.pathSpacing,'Trajectory:Path','Path is too short for the selected spacing.');
assert(s0(end)/c.pathSpacing<100000,'Trajectory:Path','Path has too many samples; increase spacing.');
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
    theta = linspace(0,2*pi*circleTurns,numel(s))'; kappa(:)=1/params.circle.radius;
end
path = struct('kind',lower(kind),'raw',original,'s',s,'xy',xy, ...
    'theta',theta,'kappa',kappa,'tangentNorm',speed,'length',s(end));
end
