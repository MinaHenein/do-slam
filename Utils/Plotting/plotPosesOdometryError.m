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


end