function a = wrap_angle(a,positivePi)
% WRAP_ANGLE Wrap residuals, with optional positive-pi legacy endpoint convention.
if nargin<2, positivePi=false; end
if positivePi
    original=a;
    a=mod(a+pi,2*pi)-pi;
    a(a==-pi & original>0)=pi;
else
    a=atan2(sin(a),cos(a));
end
end
