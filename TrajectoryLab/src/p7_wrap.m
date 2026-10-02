function a = p7_wrap(a)
% P7_WRAP Wrap an angular residual; never wrap the internal plant state.
a = atan2(sin(a),cos(a));
end
