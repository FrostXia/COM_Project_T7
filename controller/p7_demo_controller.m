function [omega,memory,info] = p7_demo_controller(qHat,ref,c,memory)
% P7_DEMO_CONTROLLER Pursuit-based wiring test; not LQR, MPC, or an optimum.
d = ref.target(1:2)-qHat(1:2);
alpha = p7_wrap(atan2(d(2),d(1))-qHat(3));
distance = max(norm(d),0.1*c.lookahead);
omega = 2*c.v0*sin(alpha)/distance;
% Resolve a target directly behind the robot with a deterministic turn.
if cos(alpha)<-0.5
    omega = sign(alpha+eps)*c.wMax;
end
omega = min(max(omega,c.wMin),c.wMax);
info = struct('ok',true,'message','DEMO pursuit controller; not the project MPC.', ...
    'predictedQ',[],'constraintResidual',NaN);
end
