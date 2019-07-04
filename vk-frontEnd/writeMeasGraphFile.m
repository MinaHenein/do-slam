function writeMeasGraphFile(frames,globalFeatures,imageRange,sequence,...
    globalCamerasGraphFileIndx,globalFeaturesGraphFileIndx, globalObjectsGraphFileIndx, noiseArray, applyNoise)

measFileName = ['vk-',num2str(sequence),'-',num2str(imageRange(1)),'-',...
    num2str(imageRange(end)),'_Meas.graph'];
fileID = fopen(strcat(pwd,'/vk-frontEnd/GraphFiles/',measFileName),'w');

odomMeasSig = [noiseArray(1) noiseArray(2) noiseArray(3) noiseArray(4) noiseArray(5) noiseArray(6)];
odomMeasCov = [noiseArray(1)^2 0.0 0.0 0.0 0.0 0.0 noiseArray(2)^2 0.0 0.0 0.0 0.0 noiseArray(3)^2 0.0 0.0 0.0,...
    noiseArray(4)^2 0.0 0.0 noiseArray(5)^2 0.0 noiseArray(6)^2];

pointMeasSig = [noiseArray(7) noiseArray(8) noiseArray(9)];
pointMeasCov = [noiseArray(7)^2 0.0 0.0 noiseArray(8)^2 0.0, noiseArray(9)^2];

for i = 1:length(frames)
    % odometry edges
    if i > 1
        label = 'EDGE_R3_SO3';
        vIn = globalCamerasGraphFileIndx(i-1);
        vOut = globalCamerasGraphFileIndx(i);
        lastCameraPose = poseToTransformationMatrix(frames(i-1).cameraPose);
        cameraPose = poseToTransformationMatrix(frames(i).cameraPose);
        odometry = transformationMatrixToPose(lastCameraPose\cameraPose);
        value = odometry';
        covariance = odomMeasCov;
        % vIn and vOut are not empty
        assert(~isempty(vIn) && ~isempty(vOut))
        % edge noise
        odomMeasNoise = normrnd([0 0 0 0 0 0],abs(odomMeasSig.*value),size([0 0 0 0 0 0]));
        % apply noise
        if applyNoise == true
            value = RelativeToAbsolutePoseR3xso3(value',odomMeasNoise');
        end
        writeEdge(label,vIn,vOut,value,covariance,fileID);
    end
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
            covariance = pointMeasCov;
            % vIn and vOut are not empty
            assert(~isempty(vIn) && ~isempty(vOut))
            % edge noise
            pointMeasNoise = normrnd([0 0 0],pointMeasSig,size([0 0 0]));
            % apply noise
            if applyNoise == true
                value = value + pointMeasNoise;
            end
            writeEdge(label,vIn,vOut,value,covariance,fileID);
            % dynamic point
            if ~globalFeatures.static(possibleFrameFeaturesIndx(j),1)
                objectIds = [globalFeatures.dynamicAssociation{:,1}];
                objectIndx = find(objectIds == globalFeatures.objectId(possibleFrameFeaturesIndx(j)));
                id2 = globalFeatures.id(possibleFrameFeaturesIndx(j));
                for k = 2:size(globalFeatures.dynamicAssociation(objectIndx,:),2)
                    associations = globalFeatures.dynamicAssociation{objectIndx,k};
                    index = find(associations ==  id2);
                    if ~isempty(index)
                        if index > 1
                            id1 = associations(index-1);
                            label = '2PointsDataAssociation';
                            vIn1 = globalFeaturesGraphFileIndx(id1);
                            vIn2 = globalFeaturesGraphFileIndx(possibleFrameFeaturesIndx(j));
                            % features are 1 frame apart
                            assert(globalFeatures.frame(id1)+1 == globalFeatures.frame(possibleFrameFeaturesIndx(j)));
                            assert(globalFeatures.static(id1)==0)
                            assert(globalFeatures.static(possibleFrameFeaturesIndx(j))==0)
                            idx1 = find(globalObjectsGraphFileIndx(:,1) == ...
                                globalFeatures.objectId(possibleFrameFeaturesIndx(j)));
                            idx2 = find(globalObjectsGraphFileIndx(:,2) == imageRange(i));
                            vOut = globalObjectsGraphFileIndx(intersect(idx1,idx2),3);
                            formatSpec = strcat('%s',repmat(' %d',1,numel(vIn1)),...
                                repmat(' %d',1,numel(vIn2)),repmat(' %d',1,numel(vOut)),'\n');
                            % vIn1, vIn2 and vOut are not empty
                            assert(~isempty(vIn1) &&  ~isempty(vIn2) && ~isempty(vOut))
                            fprintf(fileID,formatSpec,label,vIn1,vIn2,vOut);
                        end
                    end
                end
            end
        end
    end
end


end