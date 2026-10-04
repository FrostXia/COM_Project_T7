function q = sample_initial_pose(path,c,stream)
% SAMPLE_INITIAL_POSE Uniform offsets near the start and unrestricted heading.
span=max(path.xy,[],1)-min(path.xy,[],1);
% A straight path borrows the nonzero axis span for its zero-span axis.
span(span<=1e-10)=max(span);
offset=c.spawnFraction*span.*(2*rand(stream,1,2)-1);
q=[(path.xy(1,:)+offset).';-pi+2*pi*rand(stream)];
end
