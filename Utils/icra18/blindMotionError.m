pose1GT = [-0.243593 0.789251 0.325000 -0.007841 -0.002245 -0.978806]';
pose2GT = [-0.224961 0.784690 0.325000 -0.011958 -0.005534 -1.909651]';
pose3GT = [-0.212758 0.777361 0.325000 0.020140 0.009470 3.388004]';
pose4GT = [-0.214310 0.765376 0.325000 0.010181 0.008695 2.414553]';

odom12 = [0.074983 0.000000 0.000000 0.000000 0.000000 -0.075814]';
odom23 = [0.015116 0.000000 0.000000 0.000000 0.000000 -0.180058]';
odom34 = [0.000000 0.000000 0.000000 0.000000 0.000000 -0.501473]';

pose1 = [-0.243593 0.789251 0.325000 -0.007841 -0.002245 -0.978806]';
pose2 = RelativeToAbsolutePoseR3xso3(pose1,odom12);
pose3 = RelativeToAbsolutePoseR3xso3(pose2,odom23);
pose4 = RelativeToAbsolutePoseR3xso3(pose3,odom34);

%% Error
posesN = [pose1,pose2,pose3,pose4];
posesGT = [pose1GT,pose2GT,pose3GT,pose4GT];
v_rel_pose = AbsoluteToRelativePoseR3xso3(posesN(:,1),posesGT(:,1));
[ATE_translation_error,ATE_rotation_error,...
    ATE_squared_translation_error,ATE_squared_rotation_error] = ...
    Compute_AbsoluteTrajectoryError(posesN,posesGT,v_rel_pose);
%RPE
n_delta = 1;
[RPE_translation_error,RPE_rotation_error,...
    RPE_squared_translation_error,RPE_squared_rotation_error] = ...
    Compute_RelativePoseError(posesN,posesGT,v_rel_pose,n_delta);
%AARPE
[AARPE_translation_error,AARPE_rotation_error,AARPE_squared_translation_error,...
    AARPE_squared_rotation_error] = ...
    Compute_RelativePoseError_AllToAll(posesN,posesGT,v_rel_pose);

results.ATE_translation_error               = ATE_translation_error;
results.ATE_rotation_error                  = ATE_rotation_error;
results.AARPE_translation_error             = AARPE_translation_error;
results.AARPE_rotation_error                = AARPE_rotation_error;

fprintf('Absolute Trajectory Translation Error: %.4d \n',results.ATE_translation_error)
fprintf('Absolute Trajectory Rotation Error: %.4d \n',results.ATE_rotation_error)
fprintf('All to All Relative Pose Translation Error: %.4d \n',results.AARPE_translation_error)
fprintf('All to All Relative Pose Rotation Error: %.4d \n',results.AARPE_rotation_error)

