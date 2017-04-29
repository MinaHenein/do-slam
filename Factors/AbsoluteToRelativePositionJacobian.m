function [H1, H2] = AbsoluteToRelativePositionJacobian(config,poseAbsolute,positionAbsolute,positionRelative)
% Jacobian of the AbsoluteToRelativePosition Function

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
    %           perturbedAbsPose1 = RelativeToAbsolutePose(poseAbsolute,EpsPose(:,j));
    %           perturbedAbsPose1 = Relative2AbsoluteSE3(poseAbsolute,EpsPose(:,j));
    d1 = config.absoluteToRelativePointHandle(perturbedAbsPose1, positionAbsolute);
%     d1 = AbsoluteToRelativePosition(perturbedAbsPose1, positionAbsolute);
    %           d1 = AbsolutePoint2RelativePoint3D(perturbedAbsPose1,positionAbsolute);
    H1(:,j) = (d1-d)/(eps_2);
end
for j=1:dofPoint
    d2 = config.absoluteToRelativePointHandle(poseAbsolute, positionAbsolute+EpsPoint(:,j));
%     d2 = AbsoluteToRelativePosition(poseAbsolute, positionAbsolute+EpsPoint(:,j));
    %           d2 = AbsolutePoint2RelativePoint3D(poseAbsolute,positionAbsolute+EpsPoint(:,j));
    H2(:,j)= (d2-d)/(eps_2);
end
end