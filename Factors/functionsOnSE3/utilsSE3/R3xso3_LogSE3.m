function pose = R3xso3_LogSE3(poseR3xso3)
% Transforms a pose from  R^3 x se3 to Log(SE(3))

R = rot(poseR3xso3(4:6,1));
t = poseR3xso3(1:3,1);
Rt = [R t; 0 0 0 1];
pose = LogSE3(Rt);

end