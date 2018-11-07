%--------------------------------------------------------------------------
% Author: Mina Henein - mina.henein@anu.edu.au - 18/07/18
% Contributors:
%--------------------------------------------------------------------------
% VKITTI_TESTCONSTANTOBJECTMOTION
% Testing if the object motion of the objects in vKitti is constant
%--------------------------------------------------------------------------
objectPosesMatrix = 'objectCameraPoses_vKittiScene0001.mat';
objectCameraPoses = load(objectPosesMatrix);
objectCameraPoses = objectCameraPoses.objPose;

nObjects = size(objectCameraPoses,2);
Rot = [0 0 1 0;-1 0 0 0;0 -1 0 0;0 0 0 1];   

for i=1:nObjects
    figure;
    objectPoses = objectCameraPoses(i).obPose;
    cameraPoses = objectCameraPoses(i).cameraPose;
    objectPosesWorldFrame = zeros(size(objectPoses));
    for j=1:size(objectPoses,1)/4
        cameraPoseWorldFrame = Rot/cameraPoses(mapping(j,4),:)/Rot;
        objectPosesWorldFrame(mapping(j,4),:) = ...
            cameraPoseWorldFrame*objectPoses(mapping(j,4),:);
    end
    objectMotion = AbsoluteToRelativePoseR3xso3(...
        transformationMatrixToPose(objectPosesWorldFrame(mapping(1,4),:)),...
        transformationMatrixToPose(objectPosesWorldFrame(mapping(2,4),:))); 
    objectPoseFromMotion(:,1) = ...
        transformationMatrixToPose(objectPosesWorldFrame(mapping(1,4),:));
    for k=1:(size(objectPoses,1)/4) -1
        % motion is in body-fixed frame
        objectPoseFromMotion(:,k+1) = RelativeToAbsolutePoseR3xso3(...
             objectPoseFromMotion(:,k),objectMotion);
        objectNewPoseFromMotion = objectPoseFromMotion(:,k+1);
        scatter3(objectNewPoseFromMotion(1),objectNewPoseFromMotion(2),...
            objectNewPoseFromMotion(3),'r+');
        %plotCoordinates(objectNewPoseFromMotion(1:3),rot(objectNewPoseFromMotion(4:6)));
        hold on
        objectPose = transformationMatrixToPose(objectPosesWorldFrame(mapping(k+1,4),:));
        scatter3(objectPose(1),objectPose(2),objectPose(3),'bo');
        %plotCoordinates(objectPose(1:3),rot(objectPose(4:6)));
    end
end
   

% testing with the average motion transformation
constantSE3ObjectMotion = vKitti_objectMotionAveraged(objectPosesMatrix);
for i=1:nObjects
    figure;
    objectPoses = objectCameraPoses(i).obPose;
    cameraPoses = objectCameraPoses(i).cameraPose;
    objectPosesWorldFrame = zeros(size(objectPoses));
    for j=1:size(objectPoses,1)/4
        cameraPoseWorldFrame = Rot/cameraPoses(mapping(j,4),:)/Rot;
        objectPosesWorldFrame(mapping(j,4),:) = ...
            cameraPoseWorldFrame*objectPoses(mapping(j,4),:);
    end
    objectMotion = constantSE3ObjectMotion(:,i); 
    objectPoseFromMotion(:,1) = ...
        transformationMatrixToPose(objectPosesWorldFrame(mapping(1,4),:));
    for k=1:(size(objectPoses,1)/4) -1
        % motion is in global frame
        objectPoseFromMotion(:,k+1) = RelativeToAbsolutePoseR3xso3(...
             objectMotion,objectPoseFromMotion(:,k));
        objectNewPoseFromMotion = objectPoseFromMotion(:,k+1);
        scatter3(objectNewPoseFromMotion(1),objectNewPoseFromMotion(2),...
            objectNewPoseFromMotion(3),'r+');
        %plotCoordinates(objectNewPoseFromMotion(1:3),rot(objectNewPoseFromMotion(4:6)));
        hold on
        objectPose = transformationMatrixToPose(objectPosesWorldFrame(mapping(k+1,4),:));
        scatter3(objectPose(1),objectPose(2),objectPose(3),'bo');
        %plotCoordinates(objectPose(1:3),rot(objectPose(4:6)));
    end
end
   
