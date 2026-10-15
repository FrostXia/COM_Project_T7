classdef TrajectoryPath
    % TRAJECTORYPATH Related operations grouped as static methods.
    methods (Static)

        function path = generate_trajectory(kind,params,raw,c)
        % GENERATE_TRAJECTORY Convert analytic or drawn paths to a shared arc-length table.
        % params is the named c.preset structure; circle presets always use one turn.
        assert(isstruct(params) && isscalar(params),'Trajectory:Preset', ...
            'Path parameters must use the named SimulationConfig.default_config.preset structure.');
        switch lower(kind)
            case 'line'
                length=params.line.length;
                validateattributes(length,{'double'},{'scalar','finite','real','positive'});
                count=max(100,ceil(length/c.pathSpacing)+1);
                xy = [linspace(0,length,count)',zeros(count,1)];
            case 'circle'
                radius=params.circle.radius;
                validateattributes(radius,{'double'},{'scalar','finite','real','positive'});
                a = linspace(0,2*pi,max(400,ceil(2*pi*radius/c.pathSpacing)+1))';
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
                path = TrajectoryPath.smooth_drawn_trajectory(raw,c);
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
            theta = linspace(0,2*pi,numel(s))'; kappa(:)=1/params.circle.radius;
        end
        path = struct('kind',lower(kind),'raw',original,'s',s,'xy',xy, ...
            'theta',theta,'kappa',kappa,'tangentNorm',speed,'length',s(end));
        end

        function path = smooth_drawn_trajectory(raw,c)
        % SMOOTH_DRAWN_TRAJECTORY Remove drawing jitter within an explicit shape-change budget.
        % Penalize second differences, preserve endpoints in the solve, and evaluate
        % a C2 cubic spline analytically. Never clip curvature to force a pass.
        validateattributes(raw,{'double'},{'2d','ncols',2,'finite','real'});
        assert(size(raw,1)>=4,'Trajectory:Path','Draw at least four distinct points.');
        original=raw;
        raw=raw([true;vecnorm(diff(raw),2,2)>1e-6],:);
        assert(size(raw,1)>=4,'Trajectory:Path','Draw at least four distinct points.');
        u0=[0;cumsum(vecnorm(diff(raw),2,2))];
        assert(u0(end)>3*c.pathSpacing,'Trajectory:Path','The stroke is too short.');
        assert(u0(end)/c.pathSpacing<100000,'Trajectory:Path','Increase path spacing for this long stroke.');
        % Use uniform input samples so mouse speed does not set the smoothing weight.
        m=max(20,ceil(u0(end)/c.pathSpacing)+1);
        u=linspace(0,u0(end),m)';
        points=interp1(u0,raw,u,'linear');
        ds=u(2)-u(1);
        i=(1:m-2)';
        D=sparse([i;i;i],[i;i+1;i+2],[ones(m-2,1);-2*ones(m-2,1);ones(m-2,1)],m-2,m);
        penalty=D'*D;
        budget=c.maxPathDeviation;
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
            w=SimulationConfig.reference_speed(c)*kap;
            score=max([max(w/c.wMax),max(w/c.wMin),0.2/max(min(speed),1e-12)]);
            if ~c.enforcePathTurnLimit
                % Smooth mouse jitter without reshaping the stroke to meet actuator bounds.
                score=0.2/max(min(speed),1e-12);
            end
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
            error('Trajectory:ShapeBudget', ...
                'No smooth candidate stays within %.3f m of the stroke. Increase Max smoothing shift or redraw.',budget);
        end
        sd=[0;cumsum(vecnorm(diff(best.xy),2,2))];
        assert(sd(end)>3*c.pathSpacing,'Trajectory:Path','Smoothed path is too short.');
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

        function [q,w] = query_trajectory(path,s,c)
        % QUERY_TRAJECTORY Query arc length; optionally hold at the terminal pose.
        s = reshape(s,1,[]);
        sq = min(max(s,0),path.length);
        x = interp1(path.s,path.xy(:,1),sq,'linear');
        y = interp1(path.s,path.xy(:,2),sq,'linear');
        th = interp1(path.s,path.theta,sq,'linear');
        kap = interp1(path.s,path.kappa,sq,'linear');
        tail = s>path.length;
        if c.endpointHold
            kap(s>=path.length)=0;
            tail(:)=false;
        end
        x(tail) = x(tail)+(s(tail)-path.length)*cos(path.theta(end));
        y(tail) = y(tail)+(s(tail)-path.length)*sin(path.theta(end));
        kap(tail)=0;
        q = [x;y;th]; w = (SimulationConfig.reference_speed(c)*kap).';
        end

        function [ref,memory] = sample_reference_preview(q,t,path,c,memory)
        % SAMPLE_REFERENCE_PREVIEW Default local projection; replace for optimized path progress.
        if ~isempty(c.referenceFcn)
            [ref,memory] = c.referenceFcn(q,t,path,c,memory);
            return
        end
        speed=SimulationConfig.reference_speed(c);
        if strcmp(c.referenceMode,'time')
            s = speed*t;
        else
            if isempty(memory) || ~isfield(memory,'s')
                [~,idx] = min(sum((path.xy-q(1:2).').^2,2));
                s = path.s(idx);
            else
                low = memory.s;
                high = min(path.length,low+max(2*speed*c.Ts,2*c.pathSpacing));
                candidates = linspace(low,high,12);
                qp = TrajectoryPath.query_trajectory(path,candidates,c);
                [~,idx] = min(sum((qp(1:2,:)-q(1:2)).^2,1));
                s = candidates(idx);
            end
        end
        s=min(s,path.length);
        memory.s = s;
        preview=s+(0:c.N)*speed*c.Ts;
        [qr,wr] = TrajectoryPath.query_trajectory(path,preview,c);
        vr=speed*ones(c.N,1);
        if c.endpointHold
            vr=diff(min(preview,path.length)).'/c.Ts;
            wr(1:c.N)=wr(1:c.N).*(vr/speed);
        end
        ref = struct('mode',c.referenceMode,'q',qr,'w',wr(1:c.N), ...
            's',s,'t',t,'path',path,'vr',vr);
        ref.target = TrajectoryPath.query_trajectory(path,s+c.lookahead,c);
        end

        function report = check_trajectory(path,c)
        % CHECK_TRAJECTORY Check sampled geometry, not global closed-loop convergence.
        enforceTurn=c.enforcePathTurnLimit;
        w = SimulationConfig.reference_speed(c)*path.kappa;
        turnBad=w<c.wMin-1e-6 | w>c.wMax+1e-6;
        bad = ~isfinite(w) | path.tangentNorm<0.2 | (enforceTurn & turnBad);
        checkXY=path.xy; checkedW=w; checkedBad=bad;
        if isfield(path,'validation')
            checkXY=path.validation.xy;
            checkedW=SimulationConfig.reference_speed(c)*path.validation.kappa;
            checkedTurnBad=checkedW<c.wMin-1e-6 | checkedW>c.wMax+1e-6;
            checkedBad=~isfinite(checkedW) | path.validation.tangentNorm<0.2 | ...
                (enforceTurn & checkedTurnBad);
        end
        report.ok = ~any(bad) && ~any(checkedBad);
        report.bad = bad;
        report.badXY = [path.xy(bad,:);checkXY(checkedBad,:)];
        report.maxRequiredTurn = max(abs([w;checkedW]));
        report.length = path.length;
        report.turnFeasible=all(isfinite(checkedW)) && all(checkedW>=c.wMin-1e-6 & checkedW<=c.wMax+1e-6) && ~any(turnBad);
        report.warnings = {};
        if ~enforceTurn && ~report.turnFeasible
            report.warnings{end+1}=sprintf('Tight turns need %.3f rad/s; controller limits remain [%.3f, %.3f]. Testing is allowed, but tracking may be poor.', ...
                report.maxRequiredTurn,c.wMin,c.wMax);
        end
        if ~report.ok
            report.summary = sprintf('FAIL: processed path needs peak turn %.3f rad/s; allowed [%.3f, %.3f]. Check red samples.', ...
                report.maxRequiredTurn,c.wMin,c.wMax);
        else
            report.summary = sprintf('PASS: sampled geometry. Length %.2f m; peak required turn %.3f rad/s.',path.length,report.maxRequiredTurn);
        end
        % Detect nonlocal close approaches without rejecting valid crossings.
        n = numel(path.s); ids = unique(round(linspace(1,n,min(n,350))));
        near = false;
        for i=1:numel(ids)
            ds = abs(path.s(ids)-path.s(ids(i)));
            dist = vecnorm(path.xy(ids,:)-path.xy(ids(i),:),2,2);
            if any(ds>max(1,3*c.lookahead) & dist<2*c.pathSpacing)
                near = true; break
            end
        end
        if near
            report.warnings{end+1} = 'Nonlocal close sections detected; crossing order needs progress management.';
        end
        if strcmp(path.kind,'freehand')
            if isfield(path,'smoothing')
                sm=path.smoothing;
                report.warnings{end+1}=sprintf('Smoothing shift %.3f m / %.3f m budget; effective span %.2f m.', ...
                    sm.maxDeviation,sm.deviationBudget,sm.spanMetres);
                if ~report.ok
                    report.warnings{end+1}='No passing candidate found within this shape budget. Increase Max smoothing shift or redraw the tight bend.';
                end
            end
            report.warnings{end+1} = 'Check the processed curve visually. Sampled curvature checks are not a continuous-path proof.';
        end
        end

        function report = summarize_trajectory(path,c)
        % SUMMARIZE_TRAJECTORY Describe a processed path without testing feasibility.
        validateattributes(path.xy,{'double'},{'2d','ncols',2,'real','finite','nonempty'});
        report=struct('ok',true,'checked',false,'bad',false(size(path.s)), ...
            'badXY',zeros(0,2),'maxRequiredTurn',max(abs(SimulationConfig.reference_speed(c)*path.kappa)), ...
            'length',path.length,'warnings',{{}},'summary','Processed path ready for confirmation.');
        end

        function stats=compute_nominal_turn(path,v0,settings)
        % COMPUTE_NOMINAL_TURN Filter small turns and select a signed representative rate.
        % The result is a fixed design value, never a replacement for reference rates.
        if nargin<3, settings=struct('thresholdDeg',1,'fallbackDeg',1); end
        validateattributes(v0,{'double'},{'scalar','real','finite','positive'});
        validateattributes(settings.thresholdDeg,{'double'},{'scalar','real','finite','positive'});
        validateattributes(settings.fallbackDeg,{'double'},{'scalar','real','finite','positive'});
        assert(isstruct(path) && all(isfield(path,{'s','kappa'})),'Trajectory:NominalPath','Missing path geometry.');
        validateattributes(path.s,{'double'},{'vector','real','finite','nonempty'});
        validateattributes(path.kappa,{'double'},{'vector','real','finite','nonempty'});
        s=path.s(:); kappa=path.kappa(:);
        assert(numel(s)==numel(kappa) && numel(s)>=2 && all(diff(s)>0), ...
            'Trajectory:NominalPath','Path must have matching samples and increasing arc length.');
        % Equal-arc samples have equal duration at constant speed.
        uniform=linspace(s(1),s(end),numel(s))';
        rates=v0*interp1(s,kappa,uniform,'linear');
        threshold=deg2rad(settings.thresholdDeg);
        % Include exact-threshold samples despite harmless floating-point rounding.
        positive=rates>=threshold-8*eps(threshold);
        negative=rates<=-threshold+8*eps(threshold);
        np=sum(positive); nn=sum(negative);
        mp=NaN; mn=NaN;
        if np>0, mp=mean(rates(positive)); end
        if nn>0, mn=mean(rates(negative)); end
        fallback=np+nn==0;
        if fallback
            nominal=deg2rad(settings.fallbackDeg);
        else
            magnitude=(sum(abs(rates(positive)))+sum(abs(rates(negative))))/(np+nn);
            nominal=magnitude;
            if nn>np, nominal=-nominal; end
        end
        stats=struct('radiansPerSecond',nominal,'degreesPerSecond',rad2deg(nominal), ...
            'positiveCount',np,'negativeCount',nn,'positiveMean',mp,'negativeMean',mn, ...
            'sampleCount',numel(rates),'filteredCount',numel(rates)-np-nn, ...
            'usedFallback',fallback,'thresholdDeg',settings.thresholdDeg, ...
            'fallbackDeg',settings.fallbackDeg,'method','filtered-absolute-mean-majority-sign');
        end

        function q = sample_initial_pose(path,c,stream)
        % SAMPLE_INITIAL_POSE Uniform offsets near the start and unrestricted heading.
        span=max(path.xy,[],1)-min(path.xy,[],1);
        % A straight path borrows the nonzero axis span for its zero-span axis.
        span(span<=1e-10)=max(span);
        offset=c.spawnFraction*span.*(2*rand(stream,1,2)-1);
        q=[(path.xy(1,:)+offset).';-pi+2*pi*rand(stream)];
        end
    end
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
