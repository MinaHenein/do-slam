%--------------------------------------------------------------------------
% Author: Montiel Abello - montiel.abello@gmail.com - 23/05/17, Yash Vyas -
% yjvyas@gmail.com - 30/05/2017
% Contributors:
%--------------------------------------------------------------------------

function generateMeasurements(self,config)
%GENERATEMEASUREMENTSOCCLUSION very similar to generateMeasurements for the
%SimulatedEnvironmentSensor class, but includes extraction of mesh points
%for use.
%% 1. Initialise variables
% load frequently accessed variables from config
graphFileFolderPath = strcat(config.folderPath,config.sep,'Data',config.sep,config.graphFileFolderName);
if ~exist(graphFileFolderPath,'dir')
    mkdir(graphFileFolderPath)
end
gtFileID = fopen(strcat(config.folderPath,config.sep,'Data',...
                 config.sep,config.graphFileFolderName,config.sep,config.groundTruthFileName),'w');
mFileID  = fopen(strcat(config.folderPath,config.sep,'Data',...
                 config.sep,config.graphFileFolderName,config.sep,config.measurementsFileName),'w');
t      = config.t;
nSteps = numel(t);

% indexing variables
vertexCount         = 0;
cameraVertexIndexes = zeros(1,nSteps);
pointVertexIndexes = zeros(1,nSteps);

% find indexes for static and dynamic points
staticPointLogical      = self.get('points').get('static');
staticObjectLogical  = self.get('objects').get('static');
dynamicPointLogical     = ~staticPointLogical;
dynamicObjectLogical = ~staticObjectLogical;
staticPointIndexes      = find(staticPointLogical);
staticObjectIndexes  = find(staticObjectLogical);
dynamicPointIndexes     = find(dynamicPointLogical);
dynamiObjectIndexes = find(dynamicObjectLogical);

% error check for visibility
if isempty(self.pointVisibility)
    error('Visibility must be set first.');
end

%% 2. Loop over timestep, simulate observations, write to graph file
for i = 1:nSteps
    %sensor @ time t
    currentSensorPose = self.get('GP_Pose',t(i));
    vertexCount = vertexCount + 1;
    cameraVertexIndexes(i) = vertexCount;
    %WRITE VERTEX TO FILE
    label = config.poseVertexLabel;
    index = cameraVertexIndexes(i);
    switch config.poseParameterisation
        case 'R3xso3'
            value = currentSensorPose.get('R3xso3Pose');
        case 'logSE3'
            value = currentSensorPose.get('logSE3Pose');
        otherwise
            error('Error: unsupported pose parameterisation')
    end
    writeVertex(label,index,value,gtFileID);
    
    %odometry
    if i> 1
        prevSensorPose = self.get('GP_Pose',t(i-1));
        poseRelative = currentSensorPose.AbsoluteToRelativePose(prevSensorPose);
        poseRelativeNoisy = poseRelative.addNoise(config.noiseModel,zeros(size(config.stdPosePose)),config.stdPosePose);
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
        covariance = config.covPosePose;
        index1 = cameraVertexIndexes(i-1);
        index2 = cameraVertexIndexes(i);
        writeEdge(label,index1,index2,valueGT,covariance,gtFileID);
        writeEdge(label,index1,index2,valueMeas,covariance,mFileID);
    end
    
    %point observations
    for j = staticPointIndexes
        jPoint = self.get('points',j);
        jPointVisible = self.pointVisibility(j,i);
        jPointRelative = self.pointObservationRelative(j,i);
        if jPointVisible
            %check if point observed before
            if isempty(jPoint.get('vertexIndex'))
                vertexCount = vertexCount + 1;
                jPoint.set('vertexIndex',vertexCount); %*Passed by reference - changes point
                %WRITE VERTEX TO FILE
                label = config.pointVertexLabel;
                index = jPoint.get('vertexIndex');
                value = jPoint.get('R3Position',t(i));
                writeVertex(label,index,value,gtFileID);
            end
            %WRITE EDGE TO FILE
            label = config.posePointEdgeLabel;
            switch config.poseParameterisation
                case 'R3xso3'
                    jPointRelativeNoisy = jPointRelative.addNoise(config.noiseModel,zeros(size(config.stdPosePoint)),config.stdPosePoint); 
                    valueGT   = jPointRelative.get('R3Position');
                    valueMeas = jPointRelativeNoisy.get('R3Position');
                case 'logSE3'
                    jPointRelativeLogSE3      = jPoint.get('GP_Point',t(i)).AbsoluteToRelativePoint(self.get('GP_Pose',t(i)),'logSE3');
                    jPointRelativeLogSE3Noisy = jPointRelativeLogSE3.addNoise(config.noiseModel,zeros(size(config.stdPosePoint)),config.stdPosePoint); 
                    valueGT   = jPointRelativeLogSE3.get('R3Position');
                    valueMeas = jPointRelativeLogSE3Noisy.get('R3Position');
                otherwise
                    error('Error: unsupported pose parameterisation')
            end
            covariance = config.covPosePoint;
            index1 = cameraVertexIndexes(i);
            index2 = jPoint.get('vertexIndex');
            writeEdge(label,index1,index2,valueGT,covariance,gtFileID);
            writeEdge(label,index1,index2,valueMeas,covariance,mFileID);
        end
    end
    
    for j=dynamicPointIndexes
        jPoint = self.get('points',j);
        jPointVisible = self.pointVisibility(j,i);
        jPointRelative = self.pointObservationRelative(j,i);
        
        % only creates new index if visible
        if jPointVisible
        % add new vertex index to existing ones
            vertexCount = vertexCount + 1;
            vertexIndexes = [jPoint.get('vertexIndex') vertexCount];
            jPoint.set('vertexIndex',vertexIndexes); % sets new vertex
            
            label = config.pointVertexLabel;
            index = vertexIndexes(end);
            value = jPoint.get('R3Position',t(i));
            writeVertex(label,index,value,gtFileID);
            
            %WRITE SENSOR OBSERVATION EDGE TO FILE
            label = config.posePointEdgeLabel;
            switch config.poseParameterisation
                case 'R3xso3'
                    jPointRelativeNoisy = jPointRelative.addNoise(config.noiseModel,zeros(size(config.stdPosePoint)),config.stdPosePoint); 
                    valueGT   = jPointRelative.get('R3Position');
                    valueMeas = jPointRelativeNoisy.get('R3Position');
                case 'logSE3'
                    jPointRelativeLogSE3      = jPoint.get('GP_Point',t(i)).AbsoluteToRelativePoint(self.get('GP_Pose',t(i)),'logSE3');
                    jPointRelativeLogSE3Noisy = jPointRelativeLogSE3.addNoise(config.noiseModel,zeros(size(config.stdPosePoint)),config.stdPosePoint); 
                    valueGT   = jPointRelativeLogSE3.get('R3Position');
                    valueMeas = jPointRelativeLogSE3Noisy.get('R3Position');
                otherwise
                    error('Error: unsupported pose parameterisation')
            end
            covariance = config.covPosePoint;
            index1 = cameraVertexIndexes(i);
            index2 = vertexIndexes(end);
            writeEdge(label,index1,index2,valueGT,covariance,gtFileID);
            writeEdge(label,index1,index2,valueMeas,covariance,mFileID);
            
            if (i > 1) && (self.pointVisibility(j,i-1))
                % write edge between points if point was visible in
                % previous step
                switch config.pointMotionMeasurement
                    case 'point2Edge'
                        label = config.pointPointEdgeLabel;
                        covariance = config.covPointPoint;
                        index1 = vertexIndexes(end);
                        index2 = vertexIndexes(end-1);
                        valueGT = jPoint.get('R3Position',t(i))-jPoint.get('R3Position',t(i-1));
                        pointMotion = GP_Point(valueGT);
                        pointMotionNoisy = pointMotion.addNoise(config.noiseModel,zeros(size(config.stdPointPoint)),config.stdPointPoint); 
                        valueMeas = pointMotionNoisy.get('R3Position');
                        writeEdge(label,index1,index2,valueGT,covariance,gtFileID);
                        writeEdge(label,index1,index2,valueMeas,covariance,mFileID);
                    case 'point3Edge'
                        if (i > 2) && (self.pointVisibility(j,i-2)) % checks for second condition - whether last two were observed
                            index1 = vertexIndexes(end);
                            index2 = vertexIndexes(end-1);
                            index3 = vertexIndexes(end-2);
                            label = config.point3EdgeLabel;
                            if strcmp(config.motionModel,'constantSpeed')
                                valueGT = norm(jPoint.get('R3Position',t(i)) - jPoint.get('R3Position',t(i-1)))-...
                                          norm(jPoint.get('R3Position',t(i-1)) - jPoint.get('R3Position',t(i-2)));
                            elseif strcmp(config.motionModel,'constantVelocity')
                                valueGT = (jPoint.get('R3Position',t(i)) - jPoint.get('R3Position',t(i-1)))-...
                                          (jPoint.get('R3Position',t(i-1)) - jPoint.get('R3Position',t(i-2)));
                            end
                            covariance = config.cov3Points;
                            point3Motion = GP_Point(valueGT);
                            point3MotionNoisy = point3Motion.addNoise(config.noiseModel,zeros(size(config.std3Points)),config.std3Points);
                            valueMeas = point3MotionNoisy.get('R3Position');
                            writeEdge(label,[index1 index2],index3,valueGT,covariance,gtFileID);
                            writeEdge(label,[index1 index2],index3,valueMeas,covariance,mFileID);
                        end
                    case 'velocity'
                        if (i > 2) && (self.pointVisibility(j,i-2)) % checks for second condition
                            % write velocity vertex and set indexes for
                            % edges
                            vertexCount = vertexCount + 1;
                            
                            % look at constraint motion Model and implement
                            % vertex and edge
                            if strcmp(config.motionModel,'constantSpeed')
                                value = mean([norm(jPoint.get('R3Position',t(i))-jPoint.get('R3Position',t(i-1))),...
                                              norm(jPoint.get('R3Position',t(i-1))-jPoint.get('R3Position',t(i-2)))]);
                                valueGT1_2 = value-norm(jPoint.get('R3Position',t(i-1))-jPoint.get('R3Position',t(i-2)));
                                valueGT2_3 = value-norm(jPoint.get('R3Position',t(i))-jPoint.get('R3Position',t(i-1)));
                            elseif strcmp(config.motionModel,'constantVelocity')
                                value = mean([jPoint.get('R3Position',t(i))-jPoint.get('R3Position',t(i-1)),...
                                              jPoint.get('R3Position',t(i-1))-jPoint.get('R3Position',t(i-2))]);
                                valueGT1_2 = value-(jPoint.get('R3Position',t(i-1))-jPoint.get('R3Position',t(i-2)));
                                valueGT2_3 = value-(jPoint.get('R3Position',t(i))-jPoint.get('R3Position',t(i-1)));
                            end
                            
                            writeVertex(config.velocityVertexLabel,vertexCount,value,gtFileID);
                            % write velocity edges
                            % point @ time 1,2 - velocity    
                            edgeLabel = config.pointVelocityEdgeLabel;
                            index1 = vertexIndexes(end-2);
                            index2 = vertexIndexes(end-1);
                            index3 = vertexCount;
                            velocityEdge1_2 = GP_Point(valueGT1_2).addNoise(config.noiseModel,zeros(size(config.std2PointsVelocity)),config.std2PointsVelocity);
                            valueMeas1_2 = velocityEdge1_2.get('R3Position');
                            velocityEdge2_3 = GP_Point(valueGT2_3).addNoise(config.noiseModel,zeros(size(config.std2PointsVelocity)),config.std2PointsVelocity);
                            valueMeas2_3 = velocityEdge2_3.get('R3Position');
                            writeEdge(edgeLabel,[index1 index2],index3,valueGT1_2,covariance,gtFileID);
                            writeEdge(edgeLabel,[index1 index2],index3,valueMeas1_2,covariance,mFileID);
                            
                            % point @ time 2,3 - velocity
                            index1 = vertexIndexes(end-1);
                            index2 = vertexIndexes(end);
                            index3 = vertexCount;
                            writeEdge(edgeLabel,[index1 index2],index3,valueGT2_3,covariance,gtFileID);
                            writeEdge(edgeLabel,[index1 index2],index3,valueMeas2_3,covariance,mFileID);
                        end
                    case 'Off'
                    otherwise
                        error('Point motion measurement type is unidentified.')
                end
            end
        end
    end
    
    %point-plane observations
    for j = staticObjectIndexes
        jObject = self.get('objects',j);
        jPointIndexes = jObject.get('pointIndexes');
        jPointVisibility = logical(self.pointVisibility(jPointIndexes,i));
        jNVisiblePoints  = sum(jPointVisibility);
        jVisiblePointIndexes = jPointIndexes(jPointVisibility);
        %check visibility
        if jNVisiblePoints > 3
            %plane visible
            self.objectVisibility(jObject.get('index'),i) = 1;
            
            if isempty(jObject.get('vertexIndex'))
                vertexCount = vertexCount + 1;
                jObject.set('vertexIndex',vertexCount); %*Passed by reference - changes object
                %WRITE VERTEX TO FILE
                label = config.planeVertexLabel;
                index = jObject.get('vertexIndex');
                value = jObject.get('parameters',t(i));
                writeVertex(label,index,value,gtFileID);
            end
        end
        %object observed previously, create point-plane edges
        if ~isempty(jObject.get('vertexIndex'))
            for k = 1:jNVisiblePoints
                %WRITE EDGE TO FILE
                label = config.pointPlaneEdgeLabel;
                valueGT = 0;
                valueMeas = valueGT;
                covariance = config.covPointPlane;
                index1 = self.get('points',jVisiblePointIndexes(k)).get('vertexIndex');
                index2 = jObject.get('vertexIndex');
                writeEdge(label,index1,index2,valueGT,covariance,gtFileID);
                writeEdge(label,index1,index2,valueMeas,covariance,mFileID);
            end
        end
    end
    
%     for j = dynamiObjectIndexes
%         continue
%         % for dynamic objects
%     end
    
end

fclose(gtFileID);
fclose(mFileID);

end

