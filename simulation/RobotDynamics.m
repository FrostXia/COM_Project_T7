classdef RobotDynamics
    % ROBOTDYNAMICS Related operations grouped as static methods.
    methods (Static)

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

        function e=tracking_error_state(q,qr)
        % TRACKING_ERROR_STATE Reference-minus-actual error in the robot body frame.
        d=qr(1:2)-q(1:2);
        th=q(3);
        e=[cos(th)*d(1)+sin(th)*d(2);-sin(th)*d(1)+cos(th)*d(2); ...
            RobotDynamics.wrap_angle(qr(3)-th)];
        end

        function a = wrap_angle(a)
        % WRAP_ANGLE Use one atan2-based residual convention throughout the app.
        a=atan2(sin(a),cos(a));
        end

        function noise = sample_truncated_gaussian(stream,sigma,bound)
        % SAMPLE_TRUNCATED_GAUSSIAN Reject outliers rather than clip them to a boundary.
        validateattributes(sigma,{'double'},{'real','finite','nonnegative'});
        validateattributes(bound,{'double'},{'real','finite','nonnegative'});
        assert(isequal(size(sigma),size(bound)),'Trajectory:NoiseSize','Noise scales must have equal sizes.');
        noise=zeros(size(sigma));
        for j=1:numel(sigma)
            if sigma(j)==0 || bound(j)==0, continue; end
            value=sigma(j)*randn(stream);
            while abs(value)>bound(j)
                value=sigma(j)*randn(stream);
            end
            noise(j)=value;
        end
        end
    end
end
