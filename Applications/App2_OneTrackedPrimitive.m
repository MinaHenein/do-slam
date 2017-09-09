%--------------------------------------------------------------------------
% Author: Yash Vyas - yjvyas@gmail.com - 03/07/2017
% Contributors:
%--------------------------------------------------------------------------
% frontEndSolverExample
clear all 
% close all 

%% 1. Config
% time
nSteps = 51;
t0 = 0;
tN = 10;
t  = linspace(t0,tN,nSteps);

config = CameraConfig();
config = setAppConfig(config); % copy same settings for error Analysis
config.set('t',t);
% set motion model in setAppConfig function
% config.set('noiseModel','Off');
config.set('groundTruthFileName','app2_groundTruth.graph');
config.set('measurementsFileName','app2_measurements.graph');

%% 2. Generate Environment
if config.rngSeed
    rng(config.rngSeed); 
end

robotWaypoints = [0:2:tN; 0 10 15 20 15 10; 0 0 5 10 15 15; 0 0 1 2 4 2];
primitiveWaypoints = [0:2:tN; 10 20 25 20 10 5; 0 0 10 15 15 15; 0 2 1 3 5 2];

% construct trajectories
robotTrajectory = PositionModelPoseTrajectory(robotWaypoints,'R3','smoothingspline');
primitiveTrajectory = PositionModelPoseTrajectory(primitiveWaypoints,'R3','smoothingspline');
robotPose = GP_Pose([5 0 0 0 0 0]);
% robotTrajectory = RelativePoseTrajectory(primitiveTrajectory,robotPose);
% primitive2Trajectory = PositionModelPoseTrajectory(primitiveWaypoints-1,'R3','smoothingspline');

environment = Environment();
environment.addEllipsoid([1 1 2],12,'R3',primitiveTrajectory);
% environment.addEllipsoid([1 1.2 2],12,'R3',primitive2Trajectory);
% environment.addPrimitive(3*rand(3,50)-1.5,'R3',primitiveTrajectory);

%% 3. Initialise Sensor
cameraTrajectory = RelativePoseTrajectory(robotTrajectory,config.cameraRelativePose);

% standard sensor
sensor = SimulatedEnvironmentSensor();
sensor.addEnvironment(environment);
sensor.addCamera(config.fieldOfView,cameraTrajectory);
sensor.setVisibility(config);

% occlusion sensor
% sensor = SimulatedEnvironmentOcclusionSensor();
% sensor.addEnvironment(environment);
% sensor.addCamera(config.fieldOfView,cameraTrajectory);
% sensor.setVisibility(config,environment);

% figure
% spy(sensor.get('pointVisibility'));

%% 4. Plot Environment
figure
viewPoint = [-50,25];
% axisLimits = [-10,50,-10,40,-1,10];
title('Environment')
axis equal
xlabel('x')
ylabel('y')
zlabel('z')
view(viewPoint)
% axis(axisLimits)
hold on
grid on
primitiveTrajectory.plot(t,[0 0 0],'axesOFF')
cameraTrajectory.plot(t,[0 1 1],'axesOFF')
frames = sensor.plot(t,environment);
% implay(frames);

%% 5. Generate Measurements & Save to Graph File
sensor.generateMeasurements(config);

%% 6. load graph files
groundTruthCell  = graphFileToCell(config,config.groundTruthFileName);
measurementsCell = graphFileToCell(config,config.measurementsFileName);

%% 7. Manual recreation of vertices
initialCell = recreateInitialVertexes(config,measurementsCell,groundTruthCell);

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
graphN.saveGraphFile(config,'app1_results.graph');

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
resultsCell = graphFileToCell(config,'app1_results.graph');
plotGraphFile(config,resultsCell,[1 0 0])