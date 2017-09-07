%objectRobotTrajectories
% object1
object1Pose = [1 2 0 0 0 pi/3]'; % initial object pose
rotationMatrix = rot([0;0;pi/20]);
translationVector = [0.25;0;0];
object1RelativePose = [translationVector; arot(rotationMatrix)];
%SE3 motion constant in world frame
constantSE3Object1Motion = [rotationMatrix, translationVector; 0 0 0 1];
nSteps = 10;
for i=2:nSteps
    object1Pose(:,i) = RelativeToAbsolutePoseR3xso3(object1Pose(:,i-1),...
        object1RelativePose);
end
%SE3 motion in object frame
% 0^H^{-1} = 0^L_{k+1} k^H^{-1} 0^L^{-1}_{k+1}
% k^H = 0^L^{-1}_{k+1} 0^H 0^L_{k+1}
SE3Object1MotionObjFrame = zeros(6,nSteps-1);
for i=1:nSteps-1
    objectPoseTransformationMatrix = ...
        [rot(object1Pose(4:6,i+1)),object1Pose(1:3,i+1); 0 0 0 1];
    SE3MotionObjFrame = ...
        inv(objectPoseTransformationMatrix)*constantSE3Object1Motion*...
        objectPoseTransformationMatrix;
    SE3Object1MotionObjFrame(:,i) = [SE3MotionObjFrame(1:3,4);...
        arot(SE3MotionObjFrame(1:3,1:3))];
end

%objPtsRelative = {[]',[]',[]',[]'};
%objectPts = objPtsRelative;
%for j=1:size(objectPts,2)
%    objectPts{j} = RelativeToAbsolutePositionR3xso3(objectPose,...
%        repmat(objectPts{j},1,nSteps));
%end
plot2DPose(object1Pose,'r')

% object2
object2Pose = [1 1 0 0 0 -pi/30]'; % initial object pose
rotationMatrix = rot([0;0;-pi/20]);
translationVector = [-0.25;0;0];
objectRelativePose = [translationVector; arot(rotationMatrix)];
%SE3 motion constant in world frame
constantSE3Object2Motion = [rotationMatrix, translationVector; 0 0 0 1];
nSteps = 10;
for i=2:nSteps
    object2Pose(:,i) = RelativeToAbsolutePoseR3xso3(object2Pose(:,i-1),...
        objectRelativePose);
end
%SE3 motion in object frame
% 0^H^{-1} = 0^L_{k+1} k^H^{-1} 0^L^{-1}_{k+1}
% k^H = 0^L^{-1}_{k+1} 0^H 0^L_{k+1}
SE3Object2MotionObjFrame = zeros(6,nSteps-1);
for i=1:nSteps-1
    objectPoseTransformationMatrix = ...
        [rot(object2Pose(4:6,i+1)),object2Pose(1:3,i+1); 0 0 0 1];
    SE3MotionObjFrame = inv(objectPoseTransformationMatrix)*constantSE3Object2Motion*...
        objectPoseTransformationMatrix;
    SE3Object2MotionObjFrame(:,i) = [SE3MotionObjFrame(1:3,4);...
        arot(SE3MotionObjFrame(1:3,1:3))];
end
%objPtsRelative = {[]',[]',[]',[]'};
%objectPts = objPtsRelative;
%for j=1:size(objectPts,2)
%    objectPts{j} = RelativeToAbsolutePositionR3xso3(objectPose,...
%        repmat(objectPts{j},1,nSteps));
%end
plot2DPose(object2Pose,'k')

% robot
robotPose = [0.4 0.6 0 0 0 pi/4]'; % initial object pose
rotationMatrix = rot([0;0;0]);
translationVector = [0.15;0.15;0];
robotRelativePose = [translationVector; arot(rotationMatrix)];
nSteps = 5;
for i=2:nSteps
    robotPose(:,i) = RelativeToAbsolutePoseR3xso3GlobalFrame(robotPose(:,i-1),...
        robotRelativePose);
end
plot2DPose(robotPose,'m')