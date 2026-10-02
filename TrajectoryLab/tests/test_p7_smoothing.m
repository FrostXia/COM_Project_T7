function summary=test_p7_smoothing
% TEST_P7_SMOOTHING Regress mouse jitter, endpoints and real turn violations.
root=fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root,'src'));
c=p7_defaults;
x=linspace(0,10,300)';
raw=[x,0.025*sin(50*x)+0.008*sin(117*x)];
rough=c; rough.smoothingWindow=1;
before=p7_make_path('freehand',[],raw,rough);
beforeCheck=p7_check_path(before,rough);
after=p7_make_path('freehand',[],raw,c);
afterCheck=p7_check_path(after,c);
assert(~beforeCheck.ok,'Jitter regression must exercise an invalid unfiltered stroke.');
assert(afterCheck.ok,'Small mouse jitter should be smoothed into a feasible path.');
assert(after.smoothing.maxDeviation<=c.maxPathDeviation+1e-9);
assert(norm(after.xy(1,:)-raw(1,:))<1e-9 && norm(after.xy(end,:)-raw(end,:))<1e-9);
assert(all(diff(after.s)>0));

a=linspace(0,1.8*pi,400)';
rawArc=[3*sin(a),3*(1-cos(a))];
rawArc=rawArc+0.012*[sin(93*a),sin(71*a)];
arc=p7_make_path('freehand',[],rawArc,c); arcCheck=p7_check_path(arc,c);
assert(arcCheck.ok,'A broad noisy arc should pass after bounded smoothing.');

small=[0.1*sin(a),0.1*(1-cos(a))];
tight=c; tight.maxPathDeviation=0.005;
smallPath=p7_make_path('freehand',[],small,tight);
smallCheck=p7_check_path(smallPath,tight);
assert(~smallCheck.ok,'A genuinely tight bend must not be accepted by clipping curvature.');
assert(smallPath.smoothing.maxDeviation<=tight.maxPathDeviation+1e-9);
assert(~isempty(smallCheck.badXY));

summary=struct('jitterBeforePeak',beforeCheck.maxRequiredTurn, ...
    'jitterAfterPeak',afterCheck.maxRequiredTurn, ...
    'jitterShift',after.smoothing.maxDeviation,'broadArcPeak',arcCheck.maxRequiredTurn, ...
    'tightArcRejected',~smallCheck.ok);
disp(summary);
fprintf('PASS: bounded smoothing regression tests\n');
end
