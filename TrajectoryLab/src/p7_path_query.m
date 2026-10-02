function [q,w] = p7_path_query(path,s,c)
% P7_PATH_QUERY Query arc length; extend the endpoint along its tangent.
% Tangent extension supplies prediction preview, not an artificial stop.
s = reshape(s,1,[]);
sq = min(max(s,0),path.length);
x = interp1(path.s,path.xy(:,1),sq,'linear');
y = interp1(path.s,path.xy(:,2),sq,'linear');
th = interp1(path.s,path.theta,sq,'linear');
kap = interp1(path.s,path.kappa,sq,'linear');
tail = s>path.length;
x(tail) = x(tail)+(s(tail)-path.length)*cos(path.theta(end));
y(tail) = y(tail)+(s(tail)-path.length)*sin(path.theta(end));
kap(tail)=0;
q = [x;y;th]; w = (c.v0*kap).';
end
