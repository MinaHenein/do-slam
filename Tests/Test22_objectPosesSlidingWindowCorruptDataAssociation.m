%--------------------------------------------------------------------------
% Author: Mina Henein - mina.henein@anu.edu.au - 25/07/2018
% Contributors:
%--------------------------------------------------------------------------
% Test object poses sliding window and corrupted data association
%--------------------------------------------------------------------------
clear;
%% 1. Config
% time
nSteps = 121;
t0 = 0;
tN = 120;
dt = (tN-t0)/(nSteps-1);
t  = linspace(t0,tN,nSteps);

config = CameraConfig();
config = setAppConfig(config);
config.set('t',t);
config.set('nSteps',nSteps);
% config.set('noiseModel','Off');

% SE3 Motion
config.set('motionModel','constantSE3MotionDA');
config.set('std2PointsSE3Motion', [0.1,0.1,0.1]');
config.set('SE3MotionVertexInitialization','eye');
config.set('newMotionVertexPerNLandmarks',inf);
config.set('objectPosesSlidingWindow',true);
config.set('objectPosesSlidingWindowSize',inf);
config.set('newMotionVertexPerNObjectPoses',inf);

%% 2. Generate Environment
if config.rngSeed
    rng(config.rngSeed);
end

% construct primitive trajectory
primitiveInitialPose_R3xso3 = [10 0 0 0 0 0.2]';
primitiveMotion_R3xso3 = [1.5*dt; 0; 0; arot(eul2rot([0.05*dt,0,0.005*dt]))];
primitiveTrajectory = ConstantMotionDiscretePoseTrajectory(t,primitiveInitialPose_R3xso3,primitiveMotion_R3xso3,'R3xso3');

% construct  robot trajectories
sampleTimes = t(1:floor(numel(t)/5):numel(t));
sampleWaypoints = primitiveTrajectory.get('R3xso3Pose',sampleTimes);
robotWaypoints = [linspace(0,tN+3,numel(sampleTimes)+1); 0 sampleWaypoints(1,:); 0 (sampleWaypoints(2,:)+0.1); 0 (sampleWaypoints(3,:)-0.1)];
robotTrajectory = PositionModelPoseTrajectory(robotWaypoints,'R3','smoothingspline');

constantSE3ObjectMotion = primitiveTrajectory.RelativePoseGlobalFrameR3xso3(t(1),t(2));

environment = Environment();
environment.addEllipsoid([5 2 3],8,'R3',primitiveTrajectory);
nObjects = environment.nEnvironmentPrimitives;

%% 3. Initialise Sensor
cameraTrajectory = RelativePoseTrajectory(robotTrajectory,config.cameraRelativePose);

% occlusion sensor
sensor = SimulatedEnvironmentOcclusionSensor();
sensor.addEnvironment(environment);
sensor.addCamera(config.fieldOfView,cameraTrajectory);
sensor.setVisibility(config,environment);

%% 4 Generate Measurements & Save to Graph File, load graph file as well
config.set('constantSE3Motion',constantSE3ObjectMotion);
config.set('pointMotionMeasurement','point2DataAssociation');
config.set('pointsDataAssociationLabel','2PointsDataAssociation');
config.set('groundTruthFileName','Test22_slidingWindow_groundTruth.graph');
config.set('measurementsFileName','Test22_slidingWindow_measurements.graph');
sensor.generateMeasurements(config);
% Check for wrong data associations and fix if necessary
dataAssociationTest(config,config.measurementsFileName,nObjects);
dataAssociationTest(config,config.groundTruthFileName,nObjects);
writeDataAssociationObjectIndices(config,nObjects);
config.set('groundTruthFileName',...
    strcat(config.groundTruthFileName(1:end-6),'Test.graph'));
config.set('measurementsFileName',...
    strcat(config.measurementsFileName(1:end-6),'Test.graph'));
% corruptDataAssociation(config,0.15);
% config.set('measurementsFileName',...
%     strcat(config.measurementsFileName(1:end-6),'Corrupted.graph'));
measurementsCell = graphFileToCell(config,config.measurementsFileName);
groundTruthCell  = graphFileToCell(config,config.groundTruthFileName);

%% 5. Solve
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
graphN.saveGraphFile(config,'Test22_slidingWindow_results.graph');

%% 6. Error analysis
%load ground truth into graph, sort if required
graphGT = Graph(config,groundTruthCell);
fprintf('\nFinal results for SE(3) Transform:\n')
resultsSE3 = errorAnalysis(config,graphGT,graphN);