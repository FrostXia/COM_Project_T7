function qnext=pose_step(q,v,w,Ts,n)
% POSE_STEP The Q5 small-step ordering: update heading before x and y.
h=Ts/n;
qnext=q;
for i=1:n
    qnext(3)=qnext(3)+w*h;
    qnext(1)=qnext(1)+v*cos(qnext(3))*h;
    qnext(2)=qnext(2)+v*sin(qnext(3))*h;
end
% Keep global heading continuous. Only differences and innovations are wrapped.
end
