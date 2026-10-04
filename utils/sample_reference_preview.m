function [ref,memory] = sample_reference_preview(qHat,t,path,c,memory)
% SAMPLE_REFERENCE_PREVIEW Default local projection; replace for optimized path progress.
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
        qp = query_trajectory(path,candidates,c);
        [~,idx] = min(sum((qp(1:2,:)-qHat(1:2)).^2,1));
        s = candidates(idx);
    end
end
s=min(s,path.length);
memory.s = s;
preview=s+(0:c.N)*c.v0*c.Ts;
[qr,wr] = query_trajectory(path,preview,c);
vr=c.v0*ones(c.N,1);
if isfield(c,'endpointHold') && c.endpointHold
    vr=diff(min(preview,path.length)).'/c.Ts;
    wr(1:c.N)=wr(1:c.N).*(vr/c.v0);
end
ref = struct('mode',c.referenceMode,'q',qr,'w',wr(1:c.N), ...
    's',s,'t',t,'path',path,'vr',vr);
ref.target = query_trajectory(path,s+c.lookahead,c);
end
