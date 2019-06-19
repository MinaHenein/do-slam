function constantSE3ObjectMotion = vKitti_objectMotionAveraged(objectPosesMatrix)

objectCameraPoses = load(objectPosesMatrix);
objectCameraPoses = objectCameraPoses.objPoses;

nObjects = size(objectCameraPoses,2);
constantSE3ObjectMotion = zeros(6,nObjects);

% average transformation
for i=1:nObjects
    objectPoses = objectCameraPoses(i).obPose;
    cameraPoses = objectCameraPoses(i).cameraPose;
    objectPosesWorldFrame = zeros(size(objectPoses));
    for j=1:size(objectPoses,1)/4
        cameraPoseWorldFrame = inv(cameraPoses(mapping(j,4),:));
        objectPosesWorldFrame(mapping(j,4),:) = ...
            cameraPoseWorldFrame*objectPoses(mapping(j,4),:);
    end
    rotations= {};
    translations = [];
    for k=2:size(objectPoses,1)/4
        objectMotion = AbsoluteToRelativePoseR3xso3GlobalFrame(...
            transformationMatrixToPose(objectPosesWorldFrame(mapping(k-1,4),:)),...
            transformationMatrixToPose(objectPosesWorldFrame(mapping(k,4),:)));
        objectMotion = poseToTransformationMatrix(objectMotion);
        rotM = objectMotion(1:3,1:3);
        t = objectMotion(1:3,4);
        rotations{k-1} = rotM;
        translations(:,k-1) = t;
    end
    if ~isempty(rotations)
        R = rotationAveraging(rotations);
        t = mean(translations,2);
        SE3Motion = [t;arot(R)];
    else
        SE3Motion = zeros(6,1);
    end
    constantSE3ObjectMotion(:,i) = SE3Motion;
end

% from pose 1 to pose 2
% for i=1:nObjects
%     objectPoses = objectCameraPoses(i).obPose;
%     cameraPoses = objectCameraPoses(i).cameraPose;
%     objectPosesWorldFrame = zeros(size(objectPoses));
%     for j=1:size(objectPoses,1)/4
%         cameraPoseWorldFrame = Rot/cameraPoses(mapping(j,4),:)/Rot;
%         objectPosesWorldFrame(mapping(j,4),:) = ...
%             objectPoses(mapping(j,4),:)*cameraPoseWorldFrame;
%     end
%     objectMotion = AbsoluteToRelativePoseR3xso3(...
%         transformationMatrixToPose(objectPosesWorldFrame(mapping(1,4),:)),...
%         transformationMatrixToPose(objectPosesWorldFrame(mapping(2,4),:))); 
%     constantSE3ObjectMotion(:,i) = objectMotion;
% end

end