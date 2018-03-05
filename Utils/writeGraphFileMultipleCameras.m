function writeGraphFileMultipleCameras(config,sensors)

nSteps = numel(config.t);

sensorTimeStart = zeros(length(sensors),1);
for i=1:length(sensors)
    sensorPointVisibility = sensors(i).get('pointVisibility');
    [~,col] = find(sensorPointVisibility);
    sensorTimeStart(i,1) = col(1); 
end

gtFileID = fopen(strcat(config.folderPath,config.sep,'Data',...
    config.sep,config.graphFileFolderName,config.sep,...
    config.groundTruthFileName),'w');
mFileID  = fopen(strcat(config.folderPath,config.sep,'Data',...
    config.sep,config.graphFileFolderName,config.sep,...
    config.measurementsFileName),'w');

vertexCount = 0;
cameraVertexIndexes = zeros(nSteps,length(sensors),1);

for i=1:nSteps
    for j= 1:length(sensors)
        staticPointLogical      = sensors(j).get('points').get('static');
        staticPointIndexes      = find(staticPointLogical);
        dynamicPointLogical     = ~staticPointLogical;
        dynamicPointIndexes     = find(dynamicPointLogical);        
        %sensor @ time t
        currentSensorPose = sensors(j).get('GP_Pose',config.t(i));
        switch config.poseParameterisation
            case 'R3xso3'
                value = currentSensorPose.get('R3xso3Pose');
                if i > 1
                    prevValue = sensors(j).get('GP_Pose',config.t(i-1)).get('R3xso3Pose');
                end
            case 'logSE3'
                value = currentSensorPose.get('logSE3Pose');
                if i > 1
                    prevValue = sensors(j).get('GP_Pose',config.t(i-1)).get('logSE3Pose');
                end
            otherwise
                error('Error: unsupported pose parameterisation')
        end
        if i == sensorTimeStart(j,1)
            vertexCount = vertexCount + 1;
            cameraVertexIndexes(i,j,1) = vertexCount;
            %WRITE VERTEX TO FILE
            label = config.poseVertexLabel;
            index = vertexCount;
            writeVertex(label,index,value,gtFileID);
        elseif (i>1) && (i ~= sensorTimeStart(j,1)) && any(value == prevValue)
            cameraVertexIndexes(i,j,1) = cameraVertexIndexes(i-1,j,1);
        end
        
        %odometry
        if i> 1
            prevSensorPose = sensors(j).get('GP_Pose',config.t(i-1));
            poseRelative = currentSensorPose.AbsoluteToRelativePose(prevSensorPose);
            poseRelativeNoisy = poseRelative.addNoise(config.noiseModel,...
                zeros(size(config.stdPosePose)),config.stdPosePose);
            %WRITE EDGE TO FILE
            label = config.posePoseEdgeLabel;
            switch config.poseParameterisation
                case 'R3xso3'
                    valueGT   = poseRelative.get('R3xso3Pose');
                    valueMeas = poseRelativeNoisy.get('R3xso3Pose');
                case 'logSE3'
                    valueGT   = poseRelative.get('logSE3Pose');
                    valueMeas = poseRelativeNoisy.get('logSE3Pose');
                otherwise
                    error('Error: unsupported pose parameterisation')
            end
            if valueGT ~= zeros(6,1)
                covariance = config.covPosePose;
                index1 = cameraVertexIndexes(i-1,j,1);
                index2 = cameraVertexIndexes(i,j,1);
                writeEdge(label,index1,index2,valueMeas,covariance,mFileID);
            end
        end
        %point observations
        sensorPointVisibility = sensors(j).get('pointVisibility');
        sensorPointObservationRelative = sensors(j).get('pointObservationRelative');
        
        for k = staticPointIndexes
            kPoint = sensors(j).get('points',k);
            kPointVisible = sensorPointVisibility(k,i);
            kPointRelative = sensorPointObservationRelative(k,i);
            if kPointVisible
                %check if point observed before
                if isempty(kPoint.get('vertexIndex'))
                    vertexCount = vertexCount + 1;
                    kPoint.set('vertexIndex',vertexCount); %*Passed by reference - changes point
                    %WRITE VERTEX TO FILE
                    label = config.pointVertexLabel;
                    index = kPoint.get('vertexIndex');
                    value = kPoint.get('R3Position',t(i));
                    writeVertex(label,index,value,gtFileID);
                elseif (i>1) && (~sensorPointVisibility(k,i-1)) && ...
                        strcmp(config.staticDataAssociation,'Off' )
                    label = config.pointVertexLabel;
                    vertexCount = vertexCount + 1;
                    vertexIndex = [kPoint.get('vertexIndex') vertexCount];
                    kPoint.set('vertexIndex',vertexIndex);
                    index = vertexIndex(end);
                    value = kPoint.get('R3Position',t(i));
                    writeVertex(label,index,value,gtFileID);
                end
                
                %WRITE EDGE TO FILE
                label = config.posePointEdgeLabel;
                switch config.poseParameterisation
                    case 'R3xso3'
                        kPointRelativeNoisy = kPointRelative.addNoise(config.noiseModel,...
                            zeros(size(config.stdPosePoint)),config.stdPosePoint);
                        valueGT   = kPointRelative.get('R3Position');
                        valueMeas = kPointRelativeNoisy.get('R3Position');
                    case 'logSE3'
                        kPointRelativeLogSE3      = kPoint.get('GP_Point',...
                            t(i)).AbsoluteToRelativePoint(sensors(j).get('GP_Pose',t(i)),'logSE3');
                        kPointRelativeLogSE3Noisy = kPointRelativeLogSE3.addNoise(config.noiseModel,...
                            zeros(size(config.stdPosePoint)),config.stdPosePoint);
                        valueGT   = kPointRelativeLogSE3.get('R3Position');
                        valueMeas = kPointRelativeLogSE3Noisy.get('R3Position');
                    otherwise
                        error('Error: unsupported pose parameterisation')
                end
                covariance = config.covPosePoint;
                index1 = cameraVertexIndexes(i,j,1);
                vertexIndex = kPoint.get('vertexIndex');
                index2 = vertexIndex(end);
                writeEdge(label,index1,index2,valueMeas,covariance,mFileID);
            end
        end
        
        for k = dynamicPointIndexes
            kPoint = sensors(j).get('points',k);
            kPointVisible = sensorPointVisibility(k,i);
            kPointRelative = sensorPointObservationRelative(k,i);
            
            if kPointVisible
                % add new vertex index to existing ones
                vertexCount = vertexCount + 1;
                vertexIndexes = [kPoint.get('vertexIndex') vertexCount];
                kPoint.set('vertexIndex',vertexIndexes); % sets new vertex
                
                label = config.pointVertexLabel;
                index = vertexIndexes(end);
                value = kPoint.get('R3Position',config.t(i));
                writeVertex(label,index,value,gtFileID);
                
                %WRITE SENSOR OBSERVATION EDGE TO FILE
                label = config.posePointEdgeLabel;
                switch config.poseParameterisation
                    case 'R3xso3'
                        jPointRelativeNoisy = kPointRelative.addNoise(config.noiseModel,...
                            zeros(size(config.stdPosePoint)),config.stdPosePoint);
                        valueMeas = jPointRelativeNoisy.get('R3Position');
                    case 'logSE3'
                        jPointRelativeLogSE3      = kPoint.get('GP_Point',...
                            config.t(i)).AbsoluteToRelativePoint(sensors(j).get('GP_Pose',config.t(i)),'logSE3');
                        jPointRelativeLogSE3Noisy = jPointRelativeLogSE3.addNoise(config.noiseModel,...
                            zeros(size(config.stdPosePoint)),config.stdPosePoint);
                        valueMeas = jPointRelativeLogSE3Noisy.get('R3Position');
                    otherwise
                        error('Error: unsupported pose parameterisation')
                end
                covariance = config.covPosePoint;
                index1 = cameraVertexIndexes(i,j,1);
                index2 = vertexIndexes(end);
                writeEdge(label,index1,index2,valueMeas,covariance,mFileID);
                if (i > 1) && (sensorPointVisibility(k,i-1))
                    % write edge between points if point was visible in
                    % previous step by same sensor
                    switch config.pointMotionMeasurement
                        case 'point2DataAssociation'
                            label = config.pointDataAssociationLabel;
                            index1 = vertexIndexes(end-1);
                            index2 = vertexIndexes(end);
                            object = kPoint.get('objectIndexes');
                            objectIndex = sensors(j).get('objects',object(1)).get('vertexIndex');
                            fprintf(gtFileID,'%s %d %d %d\n',label,index1,index2,objectIndex(end));
                            fprintf(mFileID,'%s %d %d %d\n',label,index1,index2,objectIndex(end));
                        case 'Off'
                        otherwise
                            error('Point motion measurement type is unidentified.')
                    end
                end
            end
        end 
    end   
end
   fclose(gtFileID);
   fclose(mFileID);
end