%--------------------------------------------------------------------------
% Author: Montiel Abello - montiel.abello@gmail.com - 23/05/17
% Contributors:
%--------------------------------------------------------------------------

function generateMeasurements(self,config)
%GENERATEMEASUREMENTS simulates measurements and creates ground truth and
%measurements graph files

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
self.pointVisibility     = zeros(self.nPoints,nSteps);
self.objectVisibility    = zeros(self.nObjects,nSteps);

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
    for j = 1:self.nPoints
        jPoint = self.get('points',j);
        [jPointVisible,jPointRelative] = self.pointVisible(jPoint,t(i));
        if jPointVisible
            self.pointVisibility(j,i) = 1;
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
    
    %point-plane observations
    for j = 1:self.nObjects
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
    
end

fclose(gtFileID);
fclose(mFileID);

% clear visibility - not an intrinsic property of sensor - depends on t
% self.pointVisiblity   = [];
% self.objectVisibility = [];

end

