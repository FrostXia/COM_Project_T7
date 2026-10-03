function [theta_next,x_next,y_next]=compute_next_pose_Project7(theta,x,y,v,omega,dt)
% COMPUTE_NEXT_POSE_PROJECT7 Advance physical pose; same API as original ZIP.
% Use the Q5 heading-first 100-substep plant, also used in the noise packages.
q=p7utils.pose_step([x;y;theta],v,omega,dt,100);
theta_next=wrap(q(3));
x_next=q(1); y_next=q(2);
end
