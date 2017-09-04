%--------------------------------------------------------------------------
% Author: Yash Vyas - yjvyas@gmail.com - 19/08/17
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
setAppConfig(config); % copy same settings for error Analysis
config.set('t',t);
% config.set('noiseModel','Off');
config.set('groundTruthFileName','app6_groundTruth.graph');
config.set('measurementsFileName','app6_measurements.graph');

%% 2. Generate Environment
if config.rngSeed
    rng(config.rngSeed); 
end

robotWaypoints = [0:2:tN; 0 10 15 25 35 45; 0 2 0 -2 0 0.5; 0 0 1 2 4 2];
primitive1InitialPose_R3xso3 = [10 -5 0 0 0 0]';
primitive1Motion_R3xso3 = [1; 0; 0; arot(eul2rot([0.01,0,0]))];

primitive2InitialPose_R3xso3 = [10 5 0 0 0 0]';
primitive2Motion_R3xso3 = [1; 0; 0; arot(eul2rot([-0.01,-0.005,0]))];

% construct trajectories
robotTrajectory = PositionModelPoseTrajectory(robotWaypoints,'R3','smoothingspline');
primitive1Trajectory = ConstantMotionDiscretePoseTrajectory(t,primitive1InitialPose_R3xso3,primitive1Motion_R3xso3,'R3xso3');
primitive2Trajectory = ConstantMotionDiscretePoseTrajectory(t,primitive2InitialPose_R3xso3,primitive2Motion_R3xso3,'R3xso3');

environment = Environment();
environment.addEllipsoid([1 1 2],12,'R3',primitive1Trajectory);
environment.addEllipsoid([1 2 2],12,'R3',primitive2Trajectory);
% environment.addPrimitive(3*rand(3,50)-1.5,'R3',primitiveTrajectory);

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
viewPoint = [-35,35];
% axisLimits = [-5,50,-5,20,-5,5];
% title('Environment')
axis equal
xlabel('x')
ylabel('y')
zlabel('z')
view(viewPoint)
% axis(axisLimits)
hold on
primitive1Trajectory.plot(t,[0 0 0])
primitive2Trajectory.plot(t,[0 0 0])
cameraTrajectory.plot(t,[0 1 1])
environment.plot(t)

%% 5. Generate Measurements & Save to Graph File
sensor.generateMeasurements(config);

%% 6. load graph files
groundTruthCell  = graphFileToCell(config,config.groundTruthFileName);
measurementsCell = graphFileToCell(config,config.measurementsFileName);

%% 7. Manually recreate vertexes
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
graphN.saveGraphFile(config,'app3_results.graph');

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
resultsCell = graphFileToCell(config,'app3_results.graph');
plotGraphFile(config,resultsCell,[1 0 0])