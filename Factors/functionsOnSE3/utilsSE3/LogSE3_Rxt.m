function poseR3xso3 = LogSE3_Rxt(pose)

% Transforms a pose from Log(SE(3)) to R^3 x se3
P = ExpSE3(pose);
poseR3xso3(1:3,1) = P(1:3,4);
poseR3xso3(4:6,1) = pose(4:6,1);
end


