function e=error_state(q,qr)
% ERROR_STATE Reference-minus-actual error in the robot body frame.
d=qr(1:2)-q(1:2);
th=q(3);
e=[cos(th)*d(1)+sin(th)*d(2);-sin(th)*d(1)+cos(th)*d(2); ...
    p7_wrap(qr(3)-th)];
end
