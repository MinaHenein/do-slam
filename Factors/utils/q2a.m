function axis=quaternion2Axis(q)

theta=2*acos(q(1));

if theta==0
    axis=[0,0,0];
else
    axis=q(2:4)*theta/sin(theta/2);
%     axis=axis;
end

    