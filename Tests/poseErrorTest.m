posesGT = [0,0,0,0,0,0; 1,1,1,0,pi/2,0; 1,2,3,0,0,pi/3]';
posesN = [0.2,0,0,0,0,0; 1.3,1,1,0,pi/2,0; 1,2.4,3,0,0,pi/3]';

v_rel_pose = AbsoluteToRelativePoseR3xso3(posesGT(:,1),posesN(:,1));

%% 1. Pose Error
r_f_translation_error = 0;
r_f_rotation_error = 0;
r_f_squared_translation_error = 0;
r_f_squared_rotation_error = 0;

% number of degrees per one radian (for angular unit conversion)
f_deg_per_rad = 180/pi;

assert(isequal(size(posesN),size(posesGT)));
n = size(posesN,2);

for i = 1:n

    % put the ground truth vertex in the solution coordinate frame
    v_gt_pose = RelativeToAbsolutePoseR3xso3(posesGT(:,i),v_rel_pose); % set to R3xso3 for now
    
    % calculate the errors
    v_error = AbsoluteToRelativePoseR3xso3(posesN(:,i),v_gt_pose);% set to R3xso3 for now
    f_trans_error2 = norm(v_error(1:3))^2;
    r_f_translation_error = r_f_translation_error + sqrt(f_trans_error2);
    r_f_squared_translation_error = r_f_squared_translation_error + f_trans_error2;
    f_rot_error_degrees = wrapToPi(norm(v_error(4:6))) * f_deg_per_rad;
    r_f_rotation_error = r_f_rotation_error + f_rot_error_degrees;
    r_f_squared_rotation_error = r_f_squared_rotation_error + f_rot_error_degrees * f_rot_error_degrees;
    
end

% normalise errors by number of vertices
r_f_translation_error = r_f_translation_error/n;
r_f_rotation_error = r_f_rotation_error/n;


results.ATE_translation_error               = r_f_translation_error;
results.ATE_rotation_error                  = r_f_rotation_error;

fprintf('Absolute Trajectory Translation Error: %.3f \n',results.ATE_translation_error)
fprintf('Absolute Trajectory Rotation Error: %.3f \n',results.ATE_rotation_error)