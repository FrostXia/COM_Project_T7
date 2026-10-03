function [Ad,Bd,dd]=linear_step(wr,v0,vr,Ts)
% LINEAR_STEP Freeze the reference over one interval, then use an exponential.
A=[0 wr 0;-wr 0 vr;0 0 0]; B=[0;0;-1]; d=[vr-v0;0;0];
E=expm([A B d;zeros(2,5)]*Ts);
Ad=E(1:3,1:3); Bd=E(1:3,4); dd=E(1:3,5);
end
