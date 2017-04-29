function [F1,F2] = AbsoluteToRelativePoseJacobian(config,pose1,pose2,poseRelative)
% Jacobian of the AbsoluteToRelativePose Function

eps_ = sqrt(eps);
eps_2 = 0.5 * eps_;
dof = length(pose1);
Eps = eye(dof)*eps_2;
F1 = zeros(dof,dof);
F2 = zeros(dof,dof);

d = poseRelative;
for j=1:dof
    perturbedPose1 = config.relativeToAbsolutePoseHandle(pose1,Eps(:,j)); % implements smart plus
    perturbedPose2 = config.relativeToAbsolutePoseHandle(pose2,Eps(:,j)); % implements smart plus
%     perturbedPose = RelativeToAbsolutePose(pose1,Eps(:,j));
%     perturbedPose = Relative2AbsoluteSE3(pose1,Eps(:,j));
    d1 = config.absoluteToRelativePoseHandle(perturbedPose1,pose2);
    d2 = config.absoluteToRelativePoseHandle(pose1,perturbedPose2);
%     d1 = RelativeToAbsolutePose(perturbedPose,poseRelative);
%     d1 = Relative2AbsoluteSE3(perturbedPose,poseRelative);
    F1(:,j) = (d1-d)/(eps_2);
    F2(:,j) = (d2-d)/(eps_2);
end

end