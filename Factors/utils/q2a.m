function axis=q2a(q)

mag = norm(q(2:4));
theta=2*atan2(mag,q(1));

if theta==0
    axis=[0,0,0];
else
    axis=q(2:4)*theta/sin(theta/2);
%     axis=axis;
end

    