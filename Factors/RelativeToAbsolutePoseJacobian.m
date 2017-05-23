%--------------------------------------------------------------------------
% Author: Montiel Abello - montiel.abello@gmail.com - 23/05/17
% Contributors:
%--------------------------------------------------------------------------

function F = RelativeToAbsolutePoseJacobian(config,pose1,pose2,poseRelative)
% Jacobian of the RelativeToAbsolutePose Function

eps_ = sqrt(eps);
eps_2 = 0.5 * eps_;
dof = length(pose1);
Eps = eye(dof)*eps_2;
F = zeros(dof,dof);
d = pose2;
for j=1:dof
    perturbedPose = config.relativeToAbsolutePoseHandle(pose1,Eps(:,j));
%     perturbedPose = RelativeToAbsolutePose(pose1,Eps(:,j));
%     perturbedPose = Relative2AbsoluteSE3(pose1,Eps(:,j));
    d1 = config.relativeToAbsolutePoseHandle(perturbedPose,poseRelative);
%     d1 = RelativeToAbsolutePose(perturbedPose,poseRelative);
%     d1 = Relative2AbsoluteSE3(perturbedPose,poseRelative);
    F(:,j) = (d1-d)/(eps_2);
end


end