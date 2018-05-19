%--------------------------------------------------------------------------
% Author: Mina Henein - mina.henein@anu.edu.au - 27/04/2018
% Contributors:
%--------------------------------------------------------------------------
% 3DV Experiment 2 - a moving rig of 3 outward looking cameras, 
% is crossed by 2 objects moving in the opposite direction. 
% Extrinsic and Intrinsic calibration of cameras.
%--------------------------------------------------------------------------
%% general setup
plotEnvironment = 0;
checkVerticesCorrectness = 1;
generateIndividualSensorMeasurements = 0;
%% 1. Config
% time
nSteps = 51;
t0 = 0;
tN = 50;
dt = (tN-t0)/(nSteps-1);
t  = linspace(t0,tN,nSteps);

config = CameraConfig();
config.set('landmarkErrorToMinimize','reprojection');
config = setUnitTestConfig(config);
% noise model
config.set('t',t);
config.set('noiseModel','Off');
% intrinsics
config.set('focalLength',330);
config.set('opticalCentreX',320);
config.set('opticalCentreY',240);
% SE3 Motion
config.set('pointMotionMeasurement','point2DataAssociation');
config.set('motionModel','constantSE3MotionDA');
config.set('std2PointsSE3Motion', [0.001,0.001,0.001]');
config.set('SE3MotionVertexInitialization','translation');
config.set('newMotionVertexPerNLandmarks',inf)

%% 2. Generate Environment
if config.rngSeed
    rng(config.rngSeed); 
end

%% set up robot & cameras
camera1Pose = [3;0;0;0;0;0];
camera2Pose = [0;3;0;0;0;1.57];
camera3Pose = [0;-3;0;0;0;-1.57];

robotInitialPose_R3xso3 = [0 10 0 0 0 0]';
robotMotion_R3xso3 = [0.13*dt; 0; 0; arot(eul2rot([0.0525*dt,0,0]))];

primitive1InitialPose_R3xso3 = [0 -10 0.8 0 0 pi/2]';
primitive1Motion_R3xso3 = [-0.35*dt; -1.05*dt; 0; arot(eul2rot([0.06*dt,0,0]))];

primitive2InitialPose_R3xso3 = [0 25 0.8 0 0 -pi/2]';
primitive2Motion_R3xso3 = [0.5*dt; 0.5*dt; 0; arot(eul2rot([-0.05*dt,0,0]))];

primitive3InitialPose_R3xso3 = [25 25 0.8 0 0 -pi/2]';
primitive3Motion_R3xso3 = [-0.4*dt; -0.4*dt; 0; arot(eul2rot([0.03*dt,0,0]))];

% construct trajectories
robotTrajectory = ConstantMotionDiscretePoseTrajectory(t,robotInitialPose_R3xso3,robotMotion_R3xso3,'R3xso3');
primitive1Trajectory = ConstantMotionDiscretePoseTrajectory(t,primitive1InitialPose_R3xso3,primitive1Motion_R3xso3,'R3xso3');
primitive2Trajectory = ConstantMotionDiscretePoseTrajectory(t,primitive2InitialPose_R3xso3,primitive2Motion_R3xso3,'R3xso3');
primitive3Trajectory = ConstantMotionDiscretePoseTrajectory(t,primitive3InitialPose_R3xso3,primitive3Motion_R3xso3,'R3xso3');
constantSE3ObjectMotion = [];
constantSE3ObjectMotion(:,1) = primitive1Trajectory.RelativePoseGlobalFrameR3xso3(t(1),t(2));
constantSE3ObjectMotion(:,2) = primitive2Trajectory.RelativePoseGlobalFrameR3xso3(t(1),t(2));
constantSE3ObjectMotion(:,3) = primitive3Trajectory.RelativePoseGlobalFrameR3xso3(t(1),t(2));
config.set('constantSE3Motion',constantSE3ObjectMotion);

environment = Environment();
environment.addEllipsoid([1 1 2.5],8,'R3',primitive1Trajectory);
environment.addEllipsoid([1 1 2.5],8,'R3',primitive2Trajectory);
environment.addEllipsoid([1 1 2.5],8,'R3',primitive3Trajectory);
%% 3. Initialise Sensor
camera1RelativePose = GP_Pose(AbsoluteToRelativePoseR3xso3(robotInitialPose_R3xso3,camera1Pose));
camera2RelativePose = GP_Pose(AbsoluteToRelativePoseR3xso3(robotInitialPose_R3xso3,camera2Pose));
camera3RelativePose = GP_Pose(AbsoluteToRelativePoseR3xso3(robotInitialPose_R3xso3,camera3Pose));
camera1Trajectory = RelativePoseTrajectory(robotTrajectory,camera1RelativePose);
camera2Trajectory = RelativePoseTrajectory(robotTrajectory,camera2RelativePose);
camera3Trajectory = RelativePoseTrajectory(robotTrajectory,camera3RelativePose);
% occlusion sensor
sensor1 = SimulatedEnvironmentOcclusionSensor();
sensor1.addEnvironment(environment);
sensor1.addCamera(config.fieldOfView,camera1Trajectory);
sensor1.setVisibility(config,environment);

sensor2 = SimulatedEnvironmentOcclusionSensor();
sensor2.addEnvironment(environment);
sensor2.addCamera(config.fieldOfView,camera2Trajectory);
sensor2.setVisibility(config,environment);

sensor3 = SimulatedEnvironmentOcclusionSensor();
sensor3.addEnvironment(environment);
sensor3.addCamera(config.fieldOfView,camera3Trajectory);
sensor3.setVisibility(config,environment);

sensors = [sensor1,sensor2,sensor3];
nSensors = length(sensors);

figure
spy(sensor1.get('pointVisibility'));
figure
spy(sensor2.get('pointVisibility'));
figure
spy(sensor3.get('pointVisibility'));
%% 4. Plot Environment
if plotEnvironment
    figure
    axis equal
    xlabel('x (m)')
    ylabel('y (m)')
    zlabel('z (m)')
    % view(viewPoint)
    % axis(axisLimits)
    hold on
    grid on
    primitive1Trajectory.plot(t,[0 0 0],'axesOFF')
    primitive2Trajectory.plot(t,[0 0 0],'axesOFF')
    primitive3Trajectory.plot(t,[0 0 0],'axesOFF')
    camera1Trajectory.plot(t,[0 0 1],'axesOFF')
    camera2Trajectory.plot(t,[0 0 1],'axesOFF')
    camera3Trajectory.plot(t,[0 0 1],'axesOFF')
    % set(gcf,'Position',[0 0 1024 768]);
    frames1 = sensor1.plot(t,environment,[1 0 0]);
    frames2 = sensor2.plot(t,environment,[0 1 0]);
    frames3 = sensor3.plot(t,environment,[0 0 1]);
    implay(frames1);
    implay(frames2);
    implay(frames3);
end

if generateIndividualSensorMeasurements
    config.set('groundTruthFileName' ,'groundTruthTest20a.graph');
    config.set('measurementsFileName','measurementsTest20a.graph');
    sensor1.generateMeasurements(config);
    config.set('groundTruthFileName' ,'groundTruthTest20b.graph');
    config.set('measurementsFileName','measurementsTest20b.graph');
    sensor2.generateMeasurements(config);
    config.set('groundTruthFileName' ,'groundTruthTest20c.graph');
    config.set('measurementsFileName','measurementsTest20c.graph');
    sensor3.generateMeasurements(config);
end

config.set('groundTruthFileName' ,'groundTruthTest20.graph');
config.set('measurementsFileName','measurementsTest20.graph');

%% 5. create ground truth and measurements
groundTruthVertices = {};
groundTruthEdges = {};
vertexCount = 1;

sensorsPoses = cell(1,3);
for i=1:nSensors
sensorsPoses{1,i} = sensors(i).get('R3xso3Pose',t);
sensorsVisibility(:,:,i) = sensors(i).get('pointVisibility'); 
end

sensorVertexID = zeros(nSteps,nSensors);
for i=1:nSteps
    for j=1:nSensors
        % create vertices for camera poses
        currentVertex = struct();
        currentVertex.label = config.poseVertexLabel;
        sensorPoses = sensorsPoses{1,j};
        currentVertex.value = sensorPoses(:,i);
        currentVertex.index = vertexCount;
        groundTruthVertices{end+1} = currentVertex;
        sensorVertexID(i,j)= vertexCount;
        vertexCount = vertexCount+1;
    end
    for j=1:nSensors
        sensorPointVisibility = sensorsVisibility(:,:,j);
        for k=1:sensors(j).nPoints
            kPoint = sensors(j).get('points',k);
            % create vertex for point location
            % if point is visible at time i by sensor j
            if sensorPointVisibility(k,i)
                currentVertex = struct();
                currentVertex.label = config.pointVertexLabel;
                currentVertex.index = vertexCount;
                currentVertex.value = kPoint.get('R3Position',t(i));
                groundTruthVertices{end+1} = currentVertex;
                pointVertexID{i,j,k} = vertexCount;
                vertexCount = vertexCount+1;
            end
        end
    end
end

intrinsicVertexID = zeros(1,nSensors);
for i=1:nSensors
% create vertices for cameras intrinsics
    currentVertex = struct();
    currentVertex.label = config.intrinsicVertexLabel;
    currentVertex.value = [config.focalLength;config.opticalCentreX;config.opticalCentreY];
    currentVertex.index = vertexCount;
    groundTruthVertices{end+1} = currentVertex;
    intrinsicVertexID(1,i)= vertexCount;
    vertexCount = vertexCount+1;
     for j=1:sensors(i).nObjects
        sensors(i).get('objects',j).set('vertexIndex',j)
        objectCount = j;
    end    
end

for i=1:nSteps
    for j=1:nSensors
        if i>1
            % ground Truth edges for odometry
            currentEdge = struct();
            currentEdge.index1 = sensorVertexID(i-1,j);
            currentEdge.index2 = sensorVertexID(i,j);
            currentEdge.label = config.posePoseEdgeLabel;
            sensorPoses = sensorsPoses{1,j};
            currentEdge.value = AbsoluteToRelativePoseR3xso3(sensorPoses(:,i-1),sensorPoses(:,i));
            currentEdge.std = config.stdPosePose;
            currentEdge.cov = config.covPosePose;
            currentEdge.covUT = covToUpperTriVec(currentEdge.cov);
            groundTruthEdges{end+1} = currentEdge;
        end
        sensorPointVisibility = sensorsVisibility(:,:,j);
        sensorPointID =  pointVertexID(:,j,:);
         if (j>1)
            previousSensorPointVisibility = sensorsVisibility(:,:,j-1);
            previousSensorPointID =  pointVertexID(:,j-1,:);
        end
        for k=1:sensors(j).nPoints
            if sensorPointVisibility(k,i)
            % ground Truth ternary edges for point measurements
            currentEdge = struct();
            currentEdge.index1 = sensorVertexID(i,j);
            currentEdge.index2 = sensorPointID{i,1,k};
            currentEdge.index3 = intrinsicVertexID(1,j);
            currentEdge.label = config.posePointIntrinsicEdgeLabel;
            currentEdge.value = AbsoluteToRelativePositionR3xso3Image(sensorPoses(:,i),...
                sensors(j).get('points',k).get('R3Position',t(i)),config.intrinsics);
            currentEdge.std = config.stdPosePoint;
            currentEdge.cov = config.covPosePoint;
            currentEdge.covUT = covToUpperTriVec(currentEdge.cov);
            groundTruthEdges{end+1} = currentEdge;
            end
            if ((i>1) && sensorPointVisibility(k,i-1) && sensorPointVisibility(k,i))
                % write edge between points if point was visible in
                % previous step
                currentEdge = struct();
                currentEdge.index1 = sensorPointID{i-1,1,k};
                currentEdge.index2 = sensorPointID{i,1,k};
                object = sensors(j).get('points',k).get('objectIndexes');
                currentEdge.index3 = sensors(j).get('objects',object(1)).get('vertexIndex');
                currentEdge.label = config.pointDataAssociationLabel;
                groundTruthEdges{end+1} = currentEdge;
            end
            if ((i>1) && (j>1) && sensorPointVisibility(k,i) &&...
                    previousSensorPointVisibility(k,i-1) && ~previousSensorPointVisibility(k,i))
                % write edge between points if point was visible in
                % previous step
                currentEdge = struct();
                currentEdge.index1 = previousSensorPointID{i-1,1,k};
                currentEdge.index2 = sensorPointID{i,1,k};
                object = sensors(j).get('points',k).get('objectIndexes');
                currentEdge.index3 = sensors(j).get('objects',object(1)).get('vertexIndex');
                currentEdge.label = config.pointDataAssociationLabel;
                groundTruthEdges{end+1} = currentEdge;
            end
        end
        sensorObjectVisibility = sensors(j).get('objectVisibility');
        for k = 1:sensors(j).nObjects
            if i > 1
                if sensorObjectVisibility(k,i) && ~sensorObjectVisibility(k,i-1) && ...
                        strcmp(config.objectAssociation,'Off')
                    objectCount = objectCount + 1;
                    objectIndex = [sensors(j).get('objects',k).get('vertexIndex') objectCount];
                    sensors(j).get('objects',k).set('vertexIndex',objectIndex);
                end
            end
        end
    end
end

measurementEdges = groundTruthEdges; % copies grouthTruth to add noise
for i=1:size(measurementEdges,2) % add noise on measurements
    if strcmp(measurementEdges{i}.label,config.pointDataAssociationLabel)
        continue
    else
        if strcmp(config.noiseModel,'Gaussian')
            noisyMeasurement = normrnd(measurementEdges{i}.value,measurementEdges{i}.std);
        elseif strcmp(config.noiseModel,'Off')
            noisyMeasurement = measurementEdges{i}.value;
        end
        measurementEdges{i}.value = noisyMeasurement;
    end
end
    
groundTruthGraph = fopen(strcat(config.folderPath,config.sep,'Data',...
    config.sep,config.graphFileFolderName,config.sep,config.groundTruthFileName),'w');

for i=1:size(groundTruthVertices,2)
    vertex = groundTruthVertices{i};
    formatSpec = strcat('%s %d ',repmat(' %6.6f',1,numel(vertex.value)),'\n');
    fprintf(groundTruthGraph, formatSpec, vertex.label, vertex.index, vertex.value);
end

for i=1:size(groundTruthEdges,2)
    edge = groundTruthEdges{i};
    if strcmp(edge.label,config.pointDataAssociationLabel)
        formatSpec = '%s %d %d %d \n';
        fprintf(groundTruthGraph,formatSpec,edge.label,edge.index1,edge.index2,edge.index3);
    else 
        if isfield(edge,'index3')
            % ternary edges
            formatSpec = strcat('%s %d %d %d',repmat(' %.6f',1,numel(edge.value)),...
                repmat(' %.6f',1,numel(edge.covUT)),'\n');
            fprintf(groundTruthGraph,formatSpec,edge.label,edge.index1,edge.index2,...
                edge.index3,edge.value,edge.covUT);
        else
            %default is binary edges
            formatSpec = strcat('%s %d %d ',repmat(' %.6f',1,numel(edge.value)),...
                repmat(' %.6f',1,numel(edge.covUT)),'\n');
            fprintf(groundTruthGraph,formatSpec,edge.label,edge.index1,edge.index2,edge.value,edge.covUT);
        end
    end
end

fclose(groundTruthGraph);
measurementGraph = fopen(strcat(config.folderPath,config.sep,'Data',...
    config.sep,config.graphFileFolderName,config.sep,config.measurementsFileName),'w');

for i=1:size(measurementEdges,2)
    edge = measurementEdges{i};
    if strcmp(edge.label,config.pointDataAssociationLabel)
        formatSpec = '%s %d %d %d \n';
        fprintf(groundTruthGraph,formatSpec,edge.label,edge.index1,edge.index2,edge.index3);
    else
        if isfield(edge,'index3')
            % ternary edges
            formatSpec = strcat('%s %d %d %d',repmat(' %.6f',1,numel(edge.value)),...
                repmat(' %.6f',1,numel(edge.covUT)),'\n');
            fprintf(groundTruthGraph,formatSpec,edge.label,edge.index1,edge.index2,...
                edge.index3,edge.value,edge.covUT);
        else
            %default is binary edges
            formatSpec = strcat('%s %d %d',repmat(' %.6f',1,numel(edge.value)),...
                repmat(' %.6f',1,numel(edge.covUT)),'\n');
            fprintf(measurementGraph, formatSpec, edge.label, edge.index1,...
                edge.index2, edge.value, edge.covUT);
        end
    end
end
fclose(measurementGraph);

writeDataAssociationVerticesEdges_constantSE3Motion(config,constantSE3ObjectMotion);
groundTruthCell  = graphFileToCell(config,config.groundTruthFileName);
measurementsCell = graphFileToCell(config,config.measurementsFileName);

%% check pose vertices
if checkVerticesCorrectness
    GTGraph = Graph(config,groundTruthCell);
    checkVertices = {};
    checkVertices{1} = groundTruthVertices{1};
    checkVertices{2} = groundTruthVertices{2};
    checkVertices{3} = groundTruthVertices{3};
    groundTruthPosesVerticesIndexes = GTGraph.identifyVertices('pose');
    groundTruthOdometryEdgesIndexes = GTGraph.identifyEdges('pose-pose');
    for i=2:nSteps
        vec = mapping(i-1,nSensors);
        for j=1:nSensors
            odometryEdge = GTGraph.edges(...
                groundTruthOdometryEdgesIndexes(vec(j))).value;
            checkVertex = struct();
            checkVertex.value = RelativeToAbsolutePoseR3xso3(checkVertices{vec(j)}.value,odometryEdge);
            checkVertices{vec(j)+nSensors} = checkVertex;
            if norm(checkVertices{vec(j)+nSensors}.value - groundTruthVertices{...
                    groundTruthPosesVerticesIndexes(vec(j)+nSensors)}.value) > 1e-6
                error('Reconstructed vertices and original vertices do not match.')
            end
        end
    end
    
    %% check point vertices
    groundTruthObservationEdgesIndexes = GTGraph.identifyEdges('pose-point-intrinsic');
    for i=1:size(groundTruthObservationEdgesIndexes,1)
        observationEdge = GTGraph.edges(...
            groundTruthObservationEdgesIndexes(i)).value;
        posePointIntrinsicsIndexes = GTGraph.edges(...
            groundTruthObservationEdgesIndexes(i)).iVertices;
        checkVertex = struct();
        checkVertex.value = RelativeToAbsolutePositionR3xso3Image(...
            GTGraph.vertices(posePointIntrinsicsIndexes(1)).value,...
            observationEdge,GTGraph.vertices(posePointIntrinsicsIndexes(3)).value);
        if norm(groundTruthVertices{posePointIntrinsicsIndexes(2)}.value - checkVertex.value) > 1e-6
            error('Reconstructed vertices and original vertices do not match.')
        end
    end
end
%% 6. solve
timeStart = tic;
graph0 = Graph();
solver = graph0.process(config,measurementsCell,groundTruthCell);
solverEnd = solver(end);
totalTime = toc(timeStart);
fprintf('\nTotal time solving: %f\n',totalTime)

%get desired graphs & systems
graph0  = solverEnd.graphs(1);
graphN  = solverEnd.graphs(end);
%save results to graph file
graphN.saveGraphFile(config,'resultsTest20.graph');

graphGT = Graph(config,groundTruthCell);
results = errorAnalysis(config,graphGT,graphN);

%% plot graph files
figure
xlabel('x')
ylabel('y')
zlabel('z')
hold on
%plot groundtruth
plotGraphFileICRA(config,groundTruthCell,'groundTruth');
resultsCell = graphFileToCell(config,'resultsTest20.graph');
hold  on
plotGraphFileICRA(config,resultsCell,'solverResults',results.relPose.get('R3xso3Pose'),results.posePointsN.get('R3xso3Pose'))
