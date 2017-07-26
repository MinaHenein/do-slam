%--------------------------------------------------------------------------
% Author: Mina Henein - mina.henein@anu.edu.au - 24/07/17
% Contributors:
%--------------------------------------------------------------------------
% Test5_velocityVertex

%% general setup
% run startup first
clear all
close all

nSteps = 3;

%% config setup 
config = CameraConfig();
config = setUnitTestConfig(config);
config.set('groundTruthFileName' ,'groundTruthTest4.graph');
config.set('measurementsFileName','measurementsTest4.graph');
config.set('stdPointPoint',[0.01 0.01 0.01]');
rng(config.rngSeed);
%% set up sensor - MANUAL
sensorPose = zeros(6,nSteps);

% applies relative motion - linear velocity in forward (x) axis and 
% constant rotation about z axis
for i=2:nSteps
    rotationMatrix = eul2rot([pi/12 0 0]);
    orientationMatrix = rot(sensorPose(4:6,i));
    relativeSensorPose = [1; 0; 0; arot(orientationMatrix*rotationMatrix)];
    sensorPose(:,i) = RelativeToAbsolutePoseR3xso3(sensorPose(:,i-1),...
        relativeSensorPose);
end

%% set up object
objPtsRelative = {[0 0 0]',[1 -1 1]',[1 1 1]'};

% applies relative motion - rotation of pi/6 radians per time step about z
% axis and pi/4 radians about y axis with linear velocity of x = 1
objectPose = [5 0 0 0 0 0]'; % moved 5 forward on x axis
for i=2:nSteps
    rotationMatrix = eul2rot([pi/6 -pi/4 0]);
    objectRelativePose = [1; 0; 0; arot(rotationMatrix)];
    objectPose(:,i) = RelativeToAbsolutePoseR3xso3(objectPose(:,i-1),...
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
    % create vertex for odometry reading
    currentVertex = struct();
    currentVertex.label = config.poseVertexLabel;
    currentVertex.value = sensorPose(:,i);
    currentVertex.index = vertexCount;
    groundTruthVertices{i,1} = currentVertex;
    vertexCount = vertexCount+1;
end

for i=1:nSteps
    for j=1:size(objectPts,2)
        % create vertex for point location
        currentVertex = struct();
        currentVertex.label = config.pointVertexLabel;
        currentVertex.index = vertexCount;
        currentVertex.value = objectPts{j}(:,i);
        groundTruthVertices{i,j+1} = currentVertex;
        vertexCount = vertexCount+1;
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
        groundTruthEdges{i,1} = currentEdge;
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
        groundTruthEdges{i,j+1} = currentEdge;
    end
end

for i=3:nSteps
    for j=1:size(objectPts,2)
        currentEdge = struct();
        currentEdge.index1 = groundTruthVertices{i-2,j+1}.index;
        currentEdge.index2 = groundTruthVertices{i-1,j+1}.index;
        currentEdge.index3 = groundTruthVertices{i,j+1}.index;
        currentEdge.label = config.point3EdgeLabel;
        currentEdge.value = norm(groundTruthVertices{i,j+1}.value-...
            groundTruthVertices{i-1,j+1}.value)-...
            norm(groundTruthVertices{i-1,j+1}.value-...
            groundTruthVertices{i-2,j+1}.value);
        currentEdge.std = config.stdPoint3;
        currentEdge.cov = config.covPoint3;
        currentEdge.covUT = covToUpperTriVec(currentEdge.cov);
        groundTruthEdges{i,j+4} = currentEdge; % add to end
    end
end

measurementEdges = groundTruthEdges; % copies grouthTruth to add noise
for i=1:numel(measurementEdges) % add noise on measurements
    if ~isempty(measurementEdges{i})
        if strcmp(config.noiseModel,'Gaussian')
            noise = normrnd(measurementEdges{i}.value,measurementEdges{i}.std);
        elseif strcmp(config.noiseModel,'Off')
            noise = measurementEdges{i}.value;
        end
        measurementEdges{i}.value = noise;
    end
end

groundTruthGraph = fopen(strcat(config.folderPath,config.sep,'Data',...
    config.sep,config.graphFileFolderName,config.sep,config.groundTruthFileName),'w');
measurementGraph = fopen(strcat(config.folderPath,config.sep,'Data',...
    config.sep,config.graphFileFolderName,config.sep,config.measurementsFileName),'w');

% only done to avoid index error
groundTruthVertices{size(groundTruthEdges,1),size(groundTruthEdges,2)} = []; 
[nRows, nColumns] = size(groundTruthEdges);
for i=1:nRows
    for j=1:nColumns
        if ~isempty(groundTruthVertices{i,j})
            vertex = groundTruthVertices{i,j};
            formatSpec = strcat('%s %d ',repmat(' %6.6f',1,numel(vertex.value)),'\n');
            fprintf(groundTruthGraph, formatSpec, vertex.label, vertex.index,...
                vertex.value);
        end
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
graph0 = Graph();
solver = graph0.process(config,measurementsCell,groundTruthCell);
solverEnd = solver(end);
% 
graphN  = solverEnd.graphs(end);
graphN.saveGraphFile(config,'resultsTest4.graph');
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
resultsCell = graphFileToCell(config,'resultsTest3.graph');
plotGraph(config,graphN,[1 0 0]);

figure
subplot(1,2,1)
spy(solverEnd.systems(end).A)
subplot(1,2,2)
spy(solverEnd.systems(end).H)
