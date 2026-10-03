function [ref,memory] = p7_reference(qHat,t,path,c,memory)
% P7_REFERENCE Default local projection; replace for optimized path progress.
if ~isempty(c.referenceFcn)
    [ref,memory] = c.referenceFcn(qHat,t,path,c,memory);
    return
end
if strcmp(c.referenceMode,'time')
    s = c.v0*t;
else
    if isempty(memory) || ~isfield(memory,'s')
        [~,idx] = min(sum((path.xy-qHat(1:2).').^2,2));
        s = path.s(idx);
    else
        low = memory.s;
        high = min(path.length,low+max(2*c.v0*c.Ts,2*c.pathSpacing));
        candidates = linspace(low,high,12);
        qp = p7_path_query(path,candidates,c);
        [~,idx] = min(sum((qp(1:2,:)-qHat(1:2)).^2,1));
        s = candidates(idx);
    end
end
memory.s = s;
[qr,wr] = p7_path_query(path,s+(0:c.N)*c.v0*c.Ts,c);
ref = struct('mode',c.referenceMode,'q',qr,'w',wr(1:c.N), ...
    's',s,'t',t,'path',path);
ref.target = p7_path_query(path,s+c.lookahead,c);
end
