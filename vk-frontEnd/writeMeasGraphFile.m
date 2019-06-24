function writeMeasGraphFile(frames,globalFeatures,imageRange,sequence,...
    globalCamerasGraphFileIndx,globalFeaturesGraphFileIndx)

measFileName = ['vk-',num2str(sequence),'-',num2str(imageRange(1)),'-',...
    num2str(imageRange(end)),'-Meas.graph'];
fileID = fopen(strcat(pwd,'/vk-frontEnd/GraphFiles/',measFileName),'w');

for i = 1:length(frames)
    % find features seen in this frame
    possibleFrameFeaturesIndx = find(globalFeatures.frame <= imageRange(i));
    for j = 1:length(possibleFrameFeaturesIndx)
        % is feature seen in frame?
        possibleFeatureWeight = globalFeatures.weight(possibleFrameFeaturesIndx(j),1);
        possibleFeatureFrame  = globalFeatures.frame(possibleFrameFeaturesIndx(j),1);
        featureLastFrame = possibleFeatureFrame + possibleFeatureWeight -1;
        if featureLastFrame >= imageRange(i)
            label = 'EDGE_3D';
            vIn = globalCamerasGraphFileIndx(i);
            vOut = globalFeaturesGraphFileIndx(possibleFrameFeaturesIndx(j));
            cameraPose = poseToTransformationMatrix(frames(i).cameraPose);
            pointWorldFrame = globalFeatures.location3D(:,possibleFrameFeaturesIndx(j));
            pointCameraFrame = cameraPose\ [pointWorldFrame;1];
            value = pointCameraFrame(1:3)';
            covariance = '';
            writeEdge(label,vIn,vOut,value,covariance,fileID);
        end
    end
    %% to do:
    %'2PointsDataAssociation'
    if i > 1
        label = 'EDGE_R3_SO3';
        vIn = globalCamerasGraphFileIndx(i-1);
        vOut = globalCamerasGraphFileIndx(i);
        lastCameraPose = poseToTransformationMatrix(frames(i-1).cameraPose);
        cameraPose = poseToTransformationMatrix(frames(i).cameraPose);
        odometry = transformationMatrixToPose(lastCameraPose\cameraPose);
        value = odometry';
        covariance = '';
        writeEdge(label,vIn,vOut,value,covariance,fileID);
    end
end
            

end