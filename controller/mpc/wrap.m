function nu = wrap(alpha)
% WRAP Compatibility wrapper for the original MPC angle convention.
validateattributes(alpha,{'double'},{'real','finite'});
nu=p7_wrap(alpha,true);
end
