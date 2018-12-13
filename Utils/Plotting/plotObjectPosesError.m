function plotObjectPosesError(config,objectInitPose,objectMotionObjectFrame,solver)

%object poses
primitiveInitPose = [0 0 0 0 0 0]';
primitiveMotion_R3xso3 = objectMotionObjectFrame;
primitiveTrajectory = ConstantMotionDiscretePoseTrajectory(config.t,...
    primitiveInitPose,primitiveMotion_R3xso3,'R3xso3');
environment = Environment();
environment.addEllipsoid([5 2 3],8,'R3',primitiveTrajectory);
pts = environment.get('environmentPoints');
ptsT1 = pts.get('R3Position',1);

[~, dynamicPointsIndices] = staticDynamicPointIndices(config);
objPoses = zeros(6,config.nSteps);
for i=1:config.nSteps
    graph = solver(i).graphs(end);   
    if ~isempty(dynamicPointsIndices)
        pts = [graph.vertices(dynamicPointsIndices(mapping(i,size(pts,2)))).value];
        [objectRot, objectTr,~] = Kabsch(ptsT1,pts);
        objPoses(:,i) = [objectTr;arot(objectRot)];
    end
end

primitiveTrajectory = ConstantMotionDiscretePoseTrajectory(config.t,...
    objectInitPose,primitiveMotion_R3xso3,'R3xso3');
%% 1. Pose Error
%ATE
ATE_translation_error = zeros(1,size(objPoses,2));
ATE_rotation_error    = zeros(1,size(objPoses,2));
for i = 1:size(objPoses,2)
    pose_error = AbsoluteToRelativePoseR3xso3(objPoses(:,i),...
        primitiveTrajectory.get('R3xso3Pose',config.t(i)));
    ATE_translation_error(1,i) = norm(pose_error(1:3));
    ATE_rotation_error(1,i)    = norm(pose_error(4:6));
end
figure(1);
plot(1:size(objPoses,2),ATE_translation_error);
title('Object poses absolute translation error');
figure(2);
plot(1:size(objPoses,2),ATE_rotation_error);
title('Object poses absolute rotation error');


%% 1. Odometry Error
ATE_odometry_translation_error = zeros(1,size(objPoses,2)-1);
ATE_odometry_rotation_error    = zeros(1,size(objPoses,2)-1);
for i = 1:size(objPoses,2)-1
    odometryN = AbsoluteToRelativePoseR3xso3(objPoses(:,i),objPoses(:,i+1));
    odometryGT = AbsoluteToRelativePoseR3xso3(primitiveTrajectory.get('R3xso3Pose',config.t(i)),...
        primitiveTrajectory.get('R3xso3Pose',config.t(i+1)));
    odometry_error = AbsoluteToRelativePoseR3xso3(odometryN,odometryGT);
    ATE_odometry_translation_error(1,i) = norm(odometry_error(1:3));
    ATE_odometry_rotation_error(1,i)    = norm(odometry_error(4:6));
end
figure(3);
plot(1:size(objPoses,2)-1,ATE_odometry_translation_error);
title('Object odometry translation error');
figure(4);
plot(1:size(objPoses,2)-1,ATE_odometry_rotation_error);
title('Object odometry rotation error');
