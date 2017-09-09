%--------------------------------------------------------------------------
% Author: Yash Vyas - yjvyas@gmail.com - 03/07/2017
% Contributors:
%--------------------------------------------------------------------------
% frontEndSolverExample
clear all 
% close all 

%% 1. Config
% time
nSteps = 121;
t0 = 0;
tN = 120;
dt = (tN-t0)/(nSteps-1);
t  = linspace(t0,tN,nSteps);

config = CameraConfig();
config = setAppConfig(config); % copy same settings for error Analysis
config.set('t',t);
%config.set('noiseModel','Off');
config.set('groundTruthFileName','app5_groundTruth.graph');
config.set('measurementsFileName','app5_measurements.graph');

% SE3 Motion
config.set('pointMotionMeasurement','point2DataAssociation');
config.set('motionModel','constantSE3MotionDA');
config.set('std2PointsSE3Motion', [0.1,0.1,0.1]');

%% 2. Generate Environment
if config.rngSeed
    rng(config.rngSeed); 
end

% construct primitive trajectory
primitiveInitialPose_R3xso3 = [10 0 0 0 0 0]';
primitiveMotion_R3xso3 = [1.5*dt; 0; 0; arot(eul2rot([0.05*dt,0,0.005*dt]))];
primitiveTrajectory = ConstantMotionDiscretePoseTrajectory(t,primitiveInitialPose_R3xso3,primitiveMotion_R3xso3,'R3xso3');

% construct  robot trajectories
sampleTimes = t(1:floor(numel(t)/5):tN+1);
sampleWaypoints = primitiveTrajectory.get('R3xso3Pose',sampleTimes);
robotWaypoints = [linspace(0,tN+3,numel(sampleTimes)+1); 0 sampleWaypoints(1,:); 0 (sampleWaypoints(2,:)+0.1); 0 (sampleWaypoints(3,:)-0.1)];
robotTrajectory = PositionModelPoseTrajectory(robotWaypoints,'R3','smoothingspline');

constantSE3ObjectMotion = primitiveTrajectory.RelativePoseGlobalFrameSE3(t(1),t(2));

environment = Environment();
environment.addEllipsoid([5 2 3],8,'R3',primitiveTrajectory);

%% 3. Initialise Sensor
cameraTrajectory = RelativePoseTrajectory(robotTrajectory,config.cameraRelativePose);

% standard sensor
% sensor = SimulatedEnvironmentSensor();
% sensor.addEnvironment(environment);
% sensor.addCamera(config.fieldOfView,cameraTrajectory);
% sensor.setVisibility(config);

% occlusion sensor
sensor = SimulatedEnvironmentOcclusionSensor();
sensor.addEnvironment(environment);
sensor.addCamera(config.fieldOfView,cameraTrajectory);
sensor.setVisibility(config,environment);

figure
spy(sensor.get('pointVisibility'));

%% 4. Plot Environment
figure
hold on
grid on
axis equal
viewPoint = [-50,25];
axisLimits = [-20,50,-10,70,-5,25];
axis equal
xlabel('x')
ylabel('y')
zlabel('z')
view(viewPoint)
axis(axisLimits)
primitiveTrajectory.plot(t,[0 0 0],'axesOFF')
cameraTrajectory.plot(t,[0 1 1],'axesOFF')
frames = sensor.plot(t,environment);
% implay(frames);

%% 4.a output video
v = VideoWriter('Data/Videos/App5_sensor_environment.mp4','MPEG-4');
open(v)
writeVideo(v,frames);
close(v)

%% 5. Generate Measurements & Save to Graph File
sensor.generateMeasurements(config);
config.set('constantSE3Motion',constantSE3ObjectMotion);
writeDataAssociationVerticesEdges(config,constantSE3ObjectMotion);

%% 6. load graph files
groundTruthCell  = graphFileToCell(config,config.groundTruthFileName);
measurementsCell = graphFileToCell(config,config.measurementsFileName);

%% 7. Manual recreation of vertices
% initialCell = recreateInitialVertexes(config,measurementsCell,groundTruthCell);

%% 8. Solve
%no constraints
timeStart = tic;
graph0 = Graph();
solver = graph0.process(config,measurementsCell,groundTruthCell);
solverEnd = solver(end);
totalTime = toc(timeStart);
fprintf('\nTotal time solving: %f\n',totalTime)

%get desired graphs & systems
graph0  = solverEnd.graphs(1);
graphN  = solverEnd.graphs(end);
fprintf('\nChi-squared error: %f\n',solverEnd.systems(end).chiSquaredError)
%save results to graph file
graphN.saveGraphFile(config,'app5_results.graph');

%% 9. Error analysis
%load ground truth into graph, sort if required
graphGT = Graph(config,groundTruthCell);
results = errorAnalysis(config,graphGT,graphN);
% fprintf('Chi Squared Error: %.4d \n',solverEnd.systems.chiSquaredError)
% fprintf('Absolute Trajectory Translation Error: %.4d \n',results.ATE_translation_error)
% fprintf('Absolute Trajectory Rotation Error: %.4d \n',results.ATE_rotation_error)
% fprintf('Absolute Structure Points Error: %d \n',results.ASE_translation_error);
% fprintf('All to All Relative Pose Squared Translation Error: %.4d \n',results.AARPE_squared_translation_error)
% fprintf('All to All Relative Pose Squared Rotation Error: %.4d \n',results.AARPE_squared_rotation_error)
% fprintf('All to All Relative Point Squared Translation Error: %.4d \n',results.AARPTE_squared_translation_error)

%% 10. Plot
    %% 10.1 Plot intial, final and ground-truth solutions
%no constraints
figure
subplot(1,2,1)
spy(solverEnd.systems(end).A)
subplot(1,2,2)
spy(solverEnd.systems(end).H)

h = figure; 
xlabel('x')
ylabel('y')
zlabel('z')
hold on
view([-50,25])
%plot groundtruth
plotGraphFile(config,groundTruthCell,[0 0 1]);
%plot results
resultsCell = graphFileToCell(config,'app5_results.graph');
plotGraphFile(config,resultsCell,[1 0 0])