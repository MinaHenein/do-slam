%--------------------------------------------------------------------------
% Author: Mina Henein - mina.henein@anu.edu.au - 19/03/18
% Contributors:
%--------------------------------------------------------------------------
% generateMeasurementsMultipleCamerasTest
%--------------------------------------------------------------------------
% settings
filePath = '/home/mina/workspace/src/Git/do-slam/Data/GraphFiles/';
gtFileID = fopen(strcat(filePath,'generateMeasurementsMultipleCamerasTestGT'),'w');
mFileID = fopen(strcat(filePath,'generateMeasurementsMultipleCamerasTestMeas'),'w');

% 3 time steps
nSteps = 3;
% 2 sensors
nSensors = 2;
% 3 points
nPoints = 3;
pt1t1 = [1;2;3];
pt2t1 = [1;0;3];
pt3t1 = [4;2;5];

pt1t2 = [1.2;2.1;3];
pt2t2 = [1.5;0.5;3.5];
pt3t2 = [4.3;2;5];

pt1t3 = [1.4;2.2;3];
pt2t3 = [2;1;4];
pt3t3 = [4.6;2;5];

points = zeros(3,nPoints,nSteps);
points(:,:,1) = [pt1t1,pt2t1,pt3t1];
points(:,:,2) = [pt1t2,pt2t2,pt3t2];
points(:,:,3) = [pt1t3,pt2t3,pt3t3]; 

sensor1Pose = [0;0;0;0;0;0];
sensor1Visibility = [1 1 1;
                     1 1 1;
                     1 1 1];
sensor2Pose = [1;1;0;0;0;0];                 
sensor2Visibility = [0 0 0;
                     0 1 1;
                     0 1 1];
                 
sensorsPose = [sensor1Pose, sensor2Pose]; 
sensorsVisibility = zeros(nSteps,nPoints,nSensors);
sensorsVisibility(:,:,1) = sensor1Visibility; 
sensorsVisibility(:,:,2) = sensor2Visibility;

%random indices
cameraVertexIndexes = [1,20;1,20;1,20];

allDynamicPointVertices = cell(nPoints,1);
allDynamicPointTime = cell(nPoints,1);

vertexCount = 1;
for i=1:nSteps
    for j= 1:nSensors
        %sensor @ time t
        currentSensorPose = sensorsPose(:,j);
        %point observations
        sensorPointVisibility = sensorsVisibility(:,:,j);
        if (j>1)
            previousSensorPointVisibility = sensorsVisibility(:,:,j-1);
        end
        for k = 1:nPoints
            kPoint = points(:,k,i);
            kPointVisible = sensorPointVisibility(k,i);
            kPointRelative = AbsoluteToRelativePositionR3xso3(currentSensorPose,kPoint);
            if kPointVisible
                if j==1 || (j>1 && ~previousSensorPointVisibility(k,i))
                    % add new vertex index to existing ones
                    vertexCount = vertexCount + 1;
                    allDynamicPointVertices{k} = [allDynamicPointVertices{k} vertexCount];
                    allDynamicPointTime{k} = [allDynamicPointTime{k} i];
                    label = 'VERTEX_POINT_3D';
                    index =  allDynamicPointVertices{k}(end);
                    value = kPoint;
                    writeVertex(label,index,value,gtFileID);
                end
                %WRITE SENSOR OBSERVATION EDGE TO FILE
                label = 'EDGE_3D';
                kPointRelativeNoisy = kPointRelative;
                valueMeas = kPointRelativeNoisy;
                covariance = [0.16,0,0,0.16,0,0.16];
                index1 = cameraVertexIndexes(i,j);
                index2 = allDynamicPointVertices{k}(end);
                writeEdge(label,index1,index2,valueMeas,covariance,mFileID);
                if ((i>1) && (sensorPointVisibility(k,i-1))) ...
                        || ((i>1) && (j>1) && (previousSensorPointVisibility(k,i-1)) ...
                        && ~previousSensorPointVisibility(k,i))
                    if (allDynamicPointTime{k}(end)-allDynamicPointTime{k}(end-1)==1)
                    % write edge between points if point was visible in
                    % previous step
                    label = '2POINTS_DataAssociation';
                    index1 = allDynamicPointVertices{k}(end-1);
                    index2 = allDynamicPointVertices{k}(end);
                    %random index
                    objectIndex = 100;
                    fprintf(gtFileID,'%s %d %d %d\n',label,index1,index2,objectIndex);
                    fprintf(mFileID,'%s %d %d %d\n',label,index1,index2,objectIndex);
                    end
                end
            end
        end  
    end
end                 