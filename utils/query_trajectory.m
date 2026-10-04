function [q,w] = query_trajectory(path,s,c)
% QUERY_TRAJECTORY Query arc length; optionally hold at the terminal pose.
s = reshape(s,1,[]);
sq = min(max(s,0),path.length);
x = interp1(path.s,path.xy(:,1),sq,'linear');
y = interp1(path.s,path.xy(:,2),sq,'linear');
th = interp1(path.s,path.theta,sq,'linear');
kap = interp1(path.s,path.kappa,sq,'linear');
tail = s>path.length;
if isfield(c,'endpointHold') && c.endpointHold
    kap(s>=path.length)=0;
    tail(:)=false;
end
x(tail) = x(tail)+(s(tail)-path.length)*cos(path.theta(end));
y(tail) = y(tail)+(s(tail)-path.length)*sin(path.theta(end));
kap(tail)=0;
q = [x;y;th]; w = (c.v0*kap).';
end
