function invp = invertPoseSE3(p)

% Invert a pose p, p being an element of Log(SE(3))

% % using inverse transformations it is too slow!
% 
% id = arot(eye(3));
% invp1 = Absolute2RelativeSE3(p,[0 0 0, id']');


% using inverse of a matrix

P = ExpSE3(p);
invP = invSE3(P);
invp = LogSE3(invP);
