function axis=q2aSyms(q)

% unnormalized axis 
% q = [w  x  y z]

theta=2*acos(q(1));
axis=theta*q(2:4)/sin(theta/2);
