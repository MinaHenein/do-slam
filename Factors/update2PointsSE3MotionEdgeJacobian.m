%--------------------------------------------------------------------------
% Author: Mina Henein - mina.henein@anu.edu.au - 16/08/17
% Contributors:
%--------------------------------------------------------------------------

function J = update2PointsSE3MotionEdgeJacobian(config,point1,point2,SE3Motion)
% Jacobian of the update2PointsSE3MotionEdge Function

eps_ = sqrt(eps);
eps_2 = 0.5 * eps_;
dof = length(SE3Motion);
Eps = eye(dof)*eps_2;

SE3MotionTransformationMatrix = [rot(SE3Motion(4:6)) SE3Motion(1:3); 0 0 0 1];

if size(point1,1)==3
    edgeValue = point1 - ...
        (SE3MotionTransformationMatrix(1:3,1:3)'*point2 - ...
         (SE3MotionTransformationMatrix(1:3,1:3)'*SE3MotionTransformationMatrix(1:3,4)));
elseif size(point1,1)==4
    edgeValue = point1 - SE3MotionTransformationMatrix\point2;
end

d = edgeValue;

J = zeros(length(edgeValue),length(SE3Motion));

for j=1:dof
    perturbedSE3Motion = config.relativeToAbsolutePoseHandle(SE3Motion,Eps(:,j));     
    perturbedSE3MotionTransformationMatrix = ...
        [rot(perturbedSE3Motion(4:6)) perturbedSE3Motion(1:3);0 0 0 1];
    
    if size(point1,1)==3
    d1 = point1 - ...
        (perturbedSE3MotionTransformationMatrix(1:3,1:3)'*point2 - ...
         (perturbedSE3MotionTransformationMatrix(1:3,1:3)'*...
         perturbedSE3MotionTransformationMatrix(1:3,4)));
    elseif size(point1,1)==4
    d1 = point1 - perturbedSE3MotionTransformationMatrix\point2;
    end
    
    J(:,j) = (d1-d)/(eps_2);
end


end