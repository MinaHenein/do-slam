%--------------------------------------------------------------------------
% Author: Mina Henein - mina.henein@anu.edu.au.com - 16/04/17
% Contributors:
%--------------------------------------------------------------------------
function [H1, H2, H3] = AbsoluteToRelativePositionJacobianImage(config,poseAbsolute,...
    positionAbsolute,positionRelative,intrinsics)
%% Jacobian of the AbsoluteToRelativePosition Function
eps_ = sqrt(eps);
eps_2 = 0.5 * eps_;
dofPose = length(poseAbsolute);
dofPoint = length(positionRelative);
EpsPose = eye(dofPose)*eps_2;
EpsPoint = eye(dofPoint)*eps_2;

H1 = zeros(dofPoint,dofPose);
H2 = zeros(dofPoint,dofPoint);
d = positionRelative;
for j=1:dofPose
    perturbedAbsPose1 = config.relativeToAbsolutePoseHandle(poseAbsolute,EpsPose(:,j));
    d1 = config.absoluteToRelativePointHandle(perturbedAbsPose1, positionAbsolute,intrinsics);
    H1(:,j) = (d1-d)/(eps_2);
end
for j=1:dofPoint
    d2 = config.absoluteToRelativePointHandle(poseAbsolute, positionAbsolute+EpsPoint(:,j),intrinsics);
    H2(:,j)= (d2-d)/(eps_2);
end
H3 = config.absoluteToRelativePointHandle(poseAbsolute,positionAbsolute,intrinsics)/intrinsics;
end