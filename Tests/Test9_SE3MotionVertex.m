%--------------------------------------------------------------------------
% Author: Mina Henein - mina.henein@anu.edu.au - 10/08/17
% Contributors:
%--------------------------------------------------------------------------
% Test8_SE3MotionVertex
% l^i_k = (_kH_k+1)^-1 * l^i_k+1

%% general setup
% run startup first
% clear all
close all

apply2PtsSE3Motion = 1;
nSteps = 3;

%% config setup 
config = CameraConfig();
config.set('groundTruthFileName' ,'groundTruthTest9.graph');
config.set('measurementsFileName','measurementsTest9.graph');
config.set('motionModel','constantSE3Motion');
config.set('std2PointsSE3Motion', [0.1,0.1,0.1]');
config = setUnitTestConfig(config);
rng(config.rngSeed);

%% set up sensor - MANUAL
sensorPose = zeros(6,nSteps);

% applies relative motion - linear velocity in forward (x) axis and 
% constant rotation about z axis
for i=2:nSteps
    rotationMatrix = rot([pi/12 0 0]');
    orientationMatrix = rot(sensorPose(4:6,i));
    relativeSensorPose = [1; 0; 0; arot(orientationMatrix*rotationMatrix)];
    sensorPose(:,i) = RelativeToAbsolutePoseR3xso3(sensorPose(:,i-1),...
        relativeSensorPose);
end

%% set up object
objPtsRelative = {[0 1 0]',[1 -1 1]',[1 1 1]'};

% applies relative motion - rotation of pi/6 radians per time step about z
% axis and -pi/4 radians about y axis with linear translation of x = 1 and
% y = 2
objectPose = [5 0 0 0 0 0]'; % initial object pose - moved 5 forward on x axis
rotationMatrix = rot([pi/6;-pi/4;0]);
translationVector = [1;2;0];
objectRelativePose = [translationVector; arot(rotationMatrix)];
%SE3 motion constant in world frame
constantSE3ObjectMotion = [rotationMatrix, translationVector; 0 0 0 1];

for i=2:nSteps
    objectPose(:,i) = RelativeToAbsolutePoseR3xso3GloabalFrame(objectPose(:,i-1),...
        objectRelativePose);
end

objectPts = objPtsRelative;

for j=1:size(objectPts,2)
    objectPts{j} = RelativeToAbsolutePositionR3xso3(objectPose,...
        repmat(objectPts{j},1,nSteps));
end

%% create ground truth and measurements
groundTruthVertices = {};
groundTruthEdges = {};
vertexCount = 1;

for i=1:nSteps
    rowCount = 0;
    % create vertex for odometry reading
    currentVertex = struct();
    currentVertex.label = config.poseVertexLabel;
    currentVertex.value = sensorPose(:,i);
    currentVertex.index = vertexCount;
    groundTruthVertices{i,1} = currentVertex;
    vertexCount = vertexCount+1;
    rowCount = rowCount+1;
    for j=1:size(objectPts,2)
        % create vertex for point location
        currentVertex = struct();
        currentVertex.label = config.pointVertexLabel;
        currentVertex.index = vertexCount;
        currentVertex.value = objectPts{j}(:,i);
        groundTruthVertices{i,rowCount+1} = currentVertex;
        vertexCount = vertexCount+1;
        rowCount = rowCount+1;
    end  
end

% SE3Motion vertex -- placed as last vertex
if apply2PtsSE3Motion
    if i==nSteps
        currentVertex = struct();
        currentVertex.label = config.SE3MotionVertexLabel;
        currentVertex.index = vertexCount;
        currentVertex.value = [constantSE3ObjectMotion(1:3,4);...
            arot(constantSE3ObjectMotion(1:3,1:3))];
        groundTruthVertices{i,rowCount+1} = currentVertex;
        vertexCount = vertexCount+1;
        rowCount = rowCount+1;
    end
end

for i=1:size(groundTruthVertices,1)
    % ground Truth edges for odometry
    if i > 1
        currentEdge = struct();
        currentEdge.index1 = groundTruthVertices{i-1,1}.index;
        currentEdge.index2 = groundTruthVertices{i,1}.index;
        currentEdge.label = config.posePoseEdgeLabel;
        currentEdge.value = AbsoluteToRelativePoseR3xso3(sensorPose(:,i-1),...
            sensorPose(:,i));
        currentEdge.std = config.stdPosePose;
        currentEdge.cov = config.covPosePose;
        currentEdge.covUT = covToUpperTriVec(currentEdge.cov);
        groundTruthEdges{i,end+1} = currentEdge;
    end
    for j=1:size(objectPts,2)
        currentEdge = struct();
        currentEdge.index1 = groundTruthVertices{i,1}.index;
        currentEdge.index2 = groundTruthVertices{i,j+1}.index;
        currentEdge.label = config.posePointEdgeLabel;
        currentEdge.value = AbsoluteToRelativePositionR3xso3(sensorPose(:,i),...
            objectPts{j}(:,i));
        currentEdge.std = config.stdPosePoint;
        currentEdge.cov = config.covPosePoint;
        currentEdge.covUT = covToUpperTriVec(currentEdge.cov);
        groundTruthEdges{i,end+1} = currentEdge;        
    end
end

if apply2PtsSE3Motion
    if i == nSteps
        for j=1:size(objectPts,2)
            for l = 1:i-1
                currentEdge = struct();
                currentEdge.index1 = groundTruthVertices{l,j+1}.index;
                currentEdge.index2 = groundTruthVertices{l+1,j+1}.index;
                currentEdge.index3 = groundTruthVertices{end,end}.index;
                currentEdge.label = config.pointSE3MotionEdgeLabel;
                currentEdge.value = groundTruthVertices{l,j+1}.value -...
                    (constantSE3ObjectMotion(1:3,1:3)'*...
                    groundTruthVertices{l+1,j+1}.value -...
                    constantSE3ObjectMotion(1:3,1:3)'*constantSE3ObjectMotion(1:3,4));
                currentEdge.std = config.std2PointsSE3Motion;
                currentEdge.cov = config.cov2PointsSE3Motion;
                currentEdge.covUT = covToUpperTriVec(currentEdge.cov);
                groundTruthEdges{i,end+1} = currentEdge;
            end
        end
    end
end

measurementEdges = groundTruthEdges; % copies grouthTruth to add noise
for i=1:numel(measurementEdges) % add noise on measurements
    if ~isempty(measurementEdges{i})
        valueEdge = measurementEdges{i}.value;
        muEdge =  zeros(size(valueEdge,1),1);
        sigmaEdge = measurementEdges{i}.std;
        if strcmp(measurementEdges{i}.label,'EDGE_R3_SO3') || ...
                strcmp(measurementEdges{i}.label,'EDGE_LOG_SE3')
            measurementEdges{i}.value = ...
                addGaussianNoise(config,muEdge,sigmaEdge,valueEdge,'pose');
        else
            measurementEdges{i}.value = ...
                addGaussianNoise(config,muEdge,sigmaEdge,valueEdge);
        end
    end
end

groundTruthGraph = fopen(strcat(config.folderPath,config.sep,'Data',...
    config.sep,config.graphFileFolderName,config.sep,config.groundTruthFileName),'w');
measurementGraph = fopen(strcat(config.folderPath,config.sep,'Data',...
    config.sep,config.graphFileFolderName,config.sep,config.measurementsFileName),'w');

[nRows, nColumns] = size(groundTruthVertices);
for i=1:nRows
    for j=1:nColumns
        if ~isempty(groundTruthVertices{i,j})
            vertex = groundTruthVertices{i,j};
            formatSpec = strcat('%s %d ',repmat(' %6.6f',1,numel(vertex.value)),'\n');
            fprintf(groundTruthGraph, formatSpec, vertex.label, vertex.index,...
                vertex.value);
        end
    end
end

[nRows, nColumns] = size(groundTruthEdges);
for i=1:nRows
    for j=1:nColumns        
        if ~isempty(groundTruthEdges{i,j})
            % print groundTruth Edge
            edge = groundTruthEdges{i,j};
            formatSpec = strcat('%s %d %d',repmat(' %.6f',1,numel(edge.value)),...
                repmat(' %.6f',1,numel(edge.covUT)),'\n');
            if isfield(edge, 'index3')
                formatSpec = strcat('%s %d %d %d',repmat(' %.6f',1,numel(edge.value)),...
                    repmat(' %.6f',1,numel(edge.covUT)),'\n');
                fprintf(groundTruthGraph,formatSpec,edge.label,edge.index1,...
                    edge.index2,edge.index3,edge.value,edge.covUT);    
            else
                fprintf(groundTruthGraph,formatSpec,edge.label,edge.index1,...
                    edge.index2,edge.value,edge.covUT);
            end
            % print Measurement edge
            edge = measurementEdges{i,j};
            if isfield(edge, 'index3')
                fprintf(measurementGraph,formatSpec,edge.label,edge.index1,...
                    edge.index2,edge.index3,edge.value,edge.covUT);
            else
                fprintf(measurementGraph,formatSpec,edge.label,edge.index1,...
                    edge.index2,edge.value,edge.covUT);
            end
        end
    end
end
fclose(groundTruthGraph);
fclose(measurementGraph);

%% solver
groundTruthCell  = graphFileToCell(config,config.groundTruthFileName);
measurementsCell = graphFileToCell(config,config.measurementsFileName);
timeStart = tic;
graph0 = Graph();
solver = graph0.process(config,measurementsCell,groundTruthCell);
solverEnd = solver(end);
totalTime = toc(timeStart);
fprintf('\nTotal time solving: %f\n',totalTime)
% 
graphN  = solverEnd.graphs(end);
graphN.saveGraphFile(config,'resultsTest9.graph');
% 
graphGT = Graph(config,groundTruthCell);
results = errorAnalysis(config,graphGT,graphN);
fprintf('Chi Squared Error: %.4d \n',solverEnd.systems.chiSquaredError)
fprintf('Absolute Trajectory Translation Error: %.4d \n',results.ATE_translation_error)
fprintf('Absolute Trajectory Rotation Error: %.4d \n',results.ATE_rotation_error)
fprintf('Absolute Structure Points Error: %d \n',results.ASE_translation_error);
fprintf('All to All Relative Pose Squared Translation Error: %.4d \n',...
    results.AARPE_squared_translation_error)
fprintf('All to All Relative Pose Squared Rotation Error: %.4d \n',...
    results.AARPE_squared_rotation_error)
fprintf('All to All Relative Point Squared Translation Error: %.4d \n',...
    results.AARPTE_squared_translation_error)

%% plot graph files
% h = figure; 
axis equal;
xlabel('x')
ylabel('y')
zlabel('z')
hold on
plotGraphFile(config,groundTruthCell,[0 0 1]);
plotGraph(config,graphN,[1 0 0]);

figure
subplot(2,2,1)
spy(solverEnd.systems(end).A)
subplot(2,2,2)
spy(solverEnd.systems(end).H)
subplot(2,2,3)
spy(solverEnd.systems(end).covariance)
subplot(2,2,4)
spy(solverEnd.systems(end).covSqrtInv)