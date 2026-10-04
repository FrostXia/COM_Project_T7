function [next,F] = integrate_robot_motion(q,w,c)
% INTEGRATE_ROBOT_MOTION Exact zero-order-hold unicycle step and state Jacobian.
a = w*c.Ts/2;
if abs(a)<1e-8
    scale = 1-a*a/6;
else
    scale = sin(a)/a;
end
dx = c.v0*c.Ts*scale*cos(q(3)+a);
dy = c.v0*c.Ts*scale*sin(q(3)+a);
next = q + [dx;dy;w*c.Ts];
F = [1 0 -dy;0 1 dx;0 0 1];
end
