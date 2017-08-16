%--------------------------------------------------------------------------
% Author: Montiel Abello - mina.henein@anu.edu.au - 16/08/17
% Contributors:
%--------------------------------------------------------------------------

function J = update2PointsSE3MotionEdgeJacobian(config,point1,point2,SE3Motion)
% Jacobian of the update2PointsSE3MotionEdge Function

eps_ = sqrt(eps);
eps_2 = 0.5 * eps_;
dof = length(SE3Motion);
Eps = eye(dof)*eps_2;

SE3MotionTransformationMatrix = [rot(SE3Motion(4:6)) SE3Motion(1:3); 0 0 0 1];
edgeValue = point1 - SE3MotionTransformationMatrix\point2;
d = edgeValue;

J = zeros(length(edgeValue),length(SE3Motion));

for j=1:dof
    perturbedSE3Motion = config.relativeToAbsolutePoseHandle(SE3Motion,Eps(:,j)); 
    perturbedSE3MotionTransformationMatrix = ...
        [rot(perturbedSE3Motion(4:6)) perturbedSE3Motion(1:3);0 0 0 1];
    d1 = point1 - perturbedSE3MotionTransformationMatrix\point2;
    J(:,j) = (d1-d)/(eps_2);
end


end