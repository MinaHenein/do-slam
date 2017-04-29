function Q = rot(axis)
% ROT determine 3x3 rotation matrix Q from scaled axis representation.
% http://mathworld.wolfram.com/RodriguesRotationFormula.html
    if any(size(axis) ~= [3 1])
        error('size mismatch')
    end
    
    x = axis(1,1);
    y = axis(2,1);
    z = axis(3,1);
    
    angle = sqrt(x * x + y * y + z * z);
    
    if (angle > 0)
        x = x / angle;
        y = y / angle;
        z = z / angle;
    end
    
    s = sin(angle);
    c = cos(angle);
    
    v = 1 - c;
    xyv = x * y * v;
    yzv = y * z * v;
    xzv = x * z * v;
    
    Q = [x * x * v + c, xyv - z * s, xzv + y * s;
         xyv + z * s, y * y * v + c, yzv - x * s;
         xzv - y * s, yzv + x * s, z * z * v + c];
end

