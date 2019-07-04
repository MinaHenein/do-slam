function [globalCamerasGraphFileIndx, globalFeaturesGraphFileIndx, globalObjectsGraphFileIndx] = ...
    writeGTGraphFile(frames, globalFeatures, imageRange, sequence)

gtFileName = ['vk-',num2str(sequence),'-',num2str(imageRange(1)),'-',...
    num2str(imageRange(end)),'_GT.graph'];
fileID = fopen(strcat(pwd,'/vk-frontEnd/GraphFiles/',gtFileName),'w');

vertexCount = 0;
globalFeaturesGraphFileIndx = zeros(size(globalFeatures.location3D,2),1);
globalCamerasGraphFileIndx = zeros(length(frames),1);
globalObjectsGraphFileIndx= [];

for i = 1:length(frames)
    % camera pose vertex
    label = 'VERTEX_POSE_R3_SO3';
    vertexCount = vertexCount + 1;
    index = vertexCount;  
    value = frames(i).cameraPose';
    globalCamerasGraphFileIndx(i,1) = vertexCount;
    writeVertex(label,index,value,fileID);
    % features extracted in this frame
    frameFeaturesIndx = find(globalFeatures.frame == imageRange(i));
    for j = 1:length(frameFeaturesIndx)
        label = 'VERTEX_POINT_3D';
        vertexCount = vertexCount + 1;
        index = vertexCount;  
        value = globalFeatures.location3D(:,frameFeaturesIndx(j))';
        globalFeaturesGraphFileIndx(frameFeaturesIndx(j),1) = vertexCount;   
        writeVertex(label,index,value,fileID);
    end
    % rigid objects motion
    if i > 1
        if ~isempty(frames(i-1).objects)
            lastFrameObjectIds =  [frames(i-1).objects.id];
            for k = 1:length(frames(i).objects)
                objectId = frames(i).objects(k).id;
                [found,indx] = find(lastFrameObjectIds == objectId);
                if ~isempty(found) && frames(i).objects(k).moving
                    label = 'VERTEX_SE3Motion';
                    vertexCount = vertexCount + 1;
                    index = vertexCount;
                    currentObjectPoseWorldFrame = poseToTransformationMatrix(frames(i).objects(k).poseWorldFrame);
                    lastObjectPoseWorldFrame = poseToTransformationMatrix(frames(i-1).objects(indx).poseWorldFrame);
                    value = transformationMatrixToPose(currentObjectPoseWorldFrame/lastObjectPoseWorldFrame);
                    globalObjectsGraphFileIndx = [globalObjectsGraphFileIndx; objectId, imageRange(i), vertexCount];
                    writeVertex(label,index,value,fileID);
                end
            end
        end
    end
end

end