function [objectsGTMotion,frames] = vKitti_objectMotion(objectPosesMatrix)

objectCameraPoses = load(objectPosesMatrix);
objectCameraPoses = objectCameraPoses.objPose;

nObjects = size(objectCameraPoses,2);
Rot = eye(4);%[0 0 1 0;-1 0 0 0;0 -1 0 0;0 0 0 1];

objectsGTMotion = {};
frames = {};
for i=1:nObjects
    objectPoses = objectCameraPoses(i).obPose;
    cameraPoses = objectCameraPoses(i).cameraPose;
    objectPosesWorldFrame = zeros(size(objectPoses));
    for j=1:size(objectPoses,1)/4
        cameraPoseWorldFrame = Rot/cameraPoses(mapping(j,4),:)/Rot;
        objectPosesWorldFrame(mapping(j,4),:) = ...
            cameraPoseWorldFrame*objectPoses(mapping(j,4),:);
    end
    objectGTMotion = zeros(6,(size(objectPoses,1)/4)-1);
    for k=2:size(objectPoses,1)/4
        objectMotion = AbsoluteToRelativePoseR3xso3GlobalFrame(...
            transformationMatrixToPose(objectPosesWorldFrame(mapping(k-1,4),:)),...
            transformationMatrixToPose(objectPosesWorldFrame(mapping(k,4),:)));
        objectGTMotion(:,k-1) = objectMotion;
    end
    objectsGTMotion{i} = objectGTMotion;
    frames{i} = objectCameraPoses(i).frameNumber;
end

end