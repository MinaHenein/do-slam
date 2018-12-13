function plotPosesOdometryError(graphN,graphGT)

%poses
posesN = [graphN.vertices(graphN.identifyVertices('pose')).value];
posesGT = [graphGT.vertices(graphGT.identifyVertices('pose')).value];

v_rel_pose = AbsoluteToRelativePoseR3xso3(posesGT(:,1),posesN(:,1));

%% 1. Pose Error
%ATE
ATE_translation_error = zeros(1,size(posesN,2));
ATE_rotation_error    = zeros(1,size(posesN,2));
for i = 1:size(posesN,2)
    [absolute_translation_error,absolute_rotation_error,~,~,~] = ...
        Compute_AbsoluteTrajectoryError(posesN(:,i),posesGT(:,i),v_rel_pose);
    ATE_translation_error(1,i) = absolute_translation_error;
    ATE_rotation_error(1,i)    = absolute_rotation_error;
end
figure(1);
plot(1:size(posesN,2),ATE_translation_error);
title('Poses absolute translation error');
figure(2);
plot(1:size(posesN,2),ATE_rotation_error);
title('Poses absolute rotation error');


%RPE
n_delta = 1;
[~,~,~,~,trError,rotError] = Compute_RelativePoseError(posesN,posesGT,v_rel_pose,n_delta);
figure(3);
plot(1:size(posesN,2)-1,trError);
title('Poses relative translation error');
figure(4);
plot(1:size(posesN,2)-1,rotError);
title('Poses relative rotation error');

%odometry
odometryN = [graphN.edges(graphN.identifyEdges('pose-pose')).value];

%% 1. Odometry Error
ATE_odometry_translation_error = zeros(1,size(posesN,2)-1);
ATE_odometry_rotation_error    = zeros(1,size(posesN,2)-1);
for i = 1:size(posesN,2)-1
    odometryGT = AbsoluteToRelativePoseR3xso3(posesGT(:,i),posesGT(:,i+1));
    odometry_error = AbsoluteToRelativePoseR3xso3(odometryN(:,i),odometryGT);
    ATE_odometry_translation_error(1,i) = norm(odometry_error(1:3));
    ATE_odometry_rotation_error(1,i)    = norm(odometry_error(4:6));
end
figure(5);
plot(1:size(posesN,2)-1,ATE_odometry_translation_error);
title('odometry translation error');
figure(6);
plot(1:size(posesN,2)-1,ATE_odometry_rotation_error);
title('odometry rotation error');


end