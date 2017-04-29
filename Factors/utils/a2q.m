function q=a2q(axis)

% implements axis to quaternion

% unnormalized axis 
% q = [w  x  y z]
thu = sqrt(axis' * axis);
if thu==0
    co = 1;
    si = 0;
    q = [co; 0; 0; 0];
else
    co = cos(thu / 2);
    si = sin(thu / 2);
    q = [co; axis * (si / thu)];
end

