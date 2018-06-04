%--------------------------------------------------------------------------
% Author: Mina Henein - mina.henein@anu.edu.au - 31/05/2018
% Contributors:
%--------------------------------------------------------------------------
clear; clc;

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
config.set('staticDataAssociation','On');

% SE3 Motion
config.set('motionModel','constantSE3MotionDA');
config.set('std2PointsSE3Motion', [0.05,0.05,0.05]');
config.set('SE3MotionVertexInitialization','eye');
config.set('newMotionVertexPerNLandmarks',inf)

%% 2. Generate Environment
if config.rngSeed
    rng(config.rngSeed);
end

primitiveInitialPose_R3xso3 = [10 3 3 0 0 0.2]';
primitiveMotion_R3xso3 = [0.8*dt; 0; 0; arot(eul2rot([0.06*dt,0,0.001*dt]))];

% construct trajectories
primitiveTrajectory = ConstantMotionDiscretePoseTrajectory(t,...
    primitiveInitialPose_R3xso3,primitiveMotion_R3xso3,'R3xso3');
%constantSE3ObjectMotion = primitiveTrajectory.RelativePoseGlobalFrameR3xso3(t(1),t(2));
constantSE3ObjectMotion = zeros(6,1);

sampleTimes = t(1:floor(numel(t)/5):numel(t));
sampleWaypoints = primitiveTrajectory.get('R3xso3Pose',sampleTimes);
robotWaypoints = [linspace(0,tN+5,7); 0 sampleWaypoints(1,:); 0 ...
    (sampleWaypoints(2,:)+0.1); 0 (sampleWaypoints(3,:)-0.1)];
robotTrajectory = PositionModelPoseTrajectory(robotWaypoints,'R3','smoothingspline');

environment = Environment();
%environment.addEllipsoid([1.25 1.25 3],12,'R3',primitiveTrajectory);
nPoints = 20;
environment.addStaticPoints([30*ones(1,nPoints); 40*rand(1,nPoints); 20*rand(1,nPoints)]);
environment.addStaticPoints([30*rand(1,nPoints); 40*ones(1,nPoints); 20*rand(1,nPoints)]);

%% 3. Initialise Sensor
cameraTrajectory = RelativePoseTrajectory(robotTrajectory,config.cameraRelativePose);

% occlusion sensor
sensor = SimulatedEnvironmentOcclusionSensor();
sensor.addEnvironment(environment);
sensor.addCamera(config.fieldOfView,cameraTrajectory);
sensor.setVisibility(config,environment);

%figure
%spy(sensor.get('pointVisibility'));

%% 4. Generate Measurements & Save to Graph File, load graph file as well
config.set('constantSE3Motion',constantSE3ObjectMotion);
config.set('pointMotionMeasurement','point2DataAssociation');
config.set('groundTruthFileName','groundTruthTest21.graph');
config.set('measurementsFileName','measurementsTest21.graph');
sensor.generateMeasurementsAllDynamic(config);
writeDataAssociationVerticesEdges_constantSE3Motion(config,constantSE3ObjectMotion);
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
graphN.saveGraphFile(config,'resultsTest21.graph');

%% 6. Error analysis
%load ground truth into graph, sort if required
graphGT = Graph(config,groundTruthCell);
fprintf('\nFinal results for SE(3) Transform:\n')
resultsSE3 = errorAnalysis(config,graphGT,graphN);

%% 7. Plot
figure
subplot(1,2,1)
spy(solverEnd.systems(end).A)
subplot(1,2,2)
spy(solverEnd.systems(end).H)

h = figure;
xlabel('x (m)')
ylabel('y (m)')
zlabel('z (m)')
hold on
grid on
axis equal
axisLimits = [-8 30 0 40 -5 20];
axis(axisLimits)
view([-50,25])
%plot groundtruth
plotGraphFileICRA(config,groundTruthCell,'groundTruth');
%plot results
resultsCell = graphFileToCell(config,'resultsTest21.graph');
plotGraphFileICRA(config,resultsCell,'solverResults',...
    resultsSE3.relPose.get('R3xso3Pose'),resultsSE3.posePointsN.get('R3xso3Pose'))