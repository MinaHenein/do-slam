function q=a2qSyms(axis)

% unnormalized axis 
% q = [w  x  y z]
thu = sqrt(axis' * axis);
co = cos(thu / 2);
si = sin(thu / 2);
q = [co; axis * (si / thu)];


