function axis=quaternion2Axis(q)

theta=2*acos(q(4));

if theta==0
    axis=[0,0,0];
else
    axis=q(1:3)/sin(theta/2);
    axis=axis/norm(axis)*theta;
end

    