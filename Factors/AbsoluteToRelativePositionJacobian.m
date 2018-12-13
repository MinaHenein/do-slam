%--------------------------------------------------------------------------
% Author: Montiel Abello - montiel.abello@gmail.com - 23/05/17
% Contributors:
%--------------------------------------------------------------------------

function [H1, H2] = AbsoluteToRelativePositionJacobian(config,poseAbsolute,positionAbsolute,positionRelative)
% Jacobian of the AbsoluteToRelativePosition Function

eps_ = sqrt(eps);
eps_2 = 0.5 * eps_;
dofPose = length(poseAbsolute);
dofPoint = length(positionAbsolute);
dofEdge = length(positionRelative);
EpsPose = eye(dofPose)*eps_2;
EpsPoint = eye(dofPoint)*eps_2;

H1 = zeros(dofEdge,dofPose);
H2 = zeros(dofEdge,dofPoint);
d = positionRelative;
for j=1:dofPose
    perturbedAbsPose1 = config.relativeToAbsolutePoseHandle(poseAbsolute,EpsPose(:,j));
    %           perturbedAbsPose1 = RelativeToAbsolutePose(poseAbsolute,EpsPose(:,j));
    %           perturbedAbsPose1 = Relative2AbsoluteSE3(poseAbsolute,EpsPose(:,j));
    if strcmp(config.landmarkErrorToMinimize,'reprojectionKnownIntrinsics')
        d1 = AbsoluteToRelativePositionR3xso3Image(perturbedAbsPose1,...
            positionAbsolute,config.intrinsics,config.R);
    else
        d1 = config.absoluteToRelativePointHandle(perturbedAbsPose1, positionAbsolute);
    end
%     d1 = AbsoluteToRelativePosition(perturbedAbsPose1, positionAbsolute);
    %           d1 = AbsolutePoint2RelativePoint3D(perturbedAbsPose1,positionAbsolute);
    H1(:,j) = (d1-d)/(eps_2);
end
for j=1:dofPoint
    if strcmp(config.landmarkErrorToMinimize,'reprojectionKnownIntrinsics')
        d2 = AbsoluteToRelativePositionR3xso3Image(poseAbsolute,...
            positionAbsolute+EpsPoint(:,j),config.intrinsics,config.R);
    else
        d2 = config.absoluteToRelativePointHandle(poseAbsolute, positionAbsolute+EpsPoint(:,j));
    end
%     d2 = AbsoluteToRelativePosition(poseAbsolute, positionAbsolute+EpsPoint(:,j));
    %           d2 = AbsolutePoint2RelativePoint3D(poseAbsolute,positionAbsolute+EpsPoint(:,j));
    H2(:,j)= (d2-d)/(eps_2);
end
end