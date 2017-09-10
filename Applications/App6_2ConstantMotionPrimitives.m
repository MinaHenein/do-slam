%--------------------------------------------------------------------------
% Author: Yash Vyas - yjvyas@gmail.com - 19/08/17
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
setAppConfig(config); % copy same settings for error Analysis
config.set('t',t);
% config.set('noiseModel','Off');
config.set('groundTruthFileName','app6_groundTruth.graph');
config.set('measurementsFileName','app6_measurements.graph');

% SE3 Motion
config.set('pointMotionMeasurement','point2DataAssociation');
config.set('motionModel','constantSE3MotionDA');
config.set('std2PointsSE3Motion', [0.1,0.1,0.1]');

%% 2. Generate Environment
if config.rngSeed
    rng(config.rngSeed); 
end

robotWaypoints = [linspace(0,tN,17); ...
    0 0    0  12.5 25 25    25 12.5 0 0     0   -12.5 -25 -25   -25 -12.5 0; ...
    0 12.5 25 25   25 12.5  0  0    0 12.5  25   25    25 12.5   0   0    0; ...
    1.5*ones(1,17)];
primitive1InitialPose_R3xso3 = [-2.5 12.5 0.8 0 0 pi/2]';
primitive1Motion_R3xso3 = [1*dt; 0; 0; arot(eul2rot([0.1*dt,0,0]))];

primitive2InitialPose_R3xso3 = [2.5 12.5 0.8 0 0 -pi/2]';
primitive2Motion_R3xso3 = [-1*dt; 0; 0; arot(eul2rot([-0.1*dt,0,0]))];

% construct trajectories
robotTrajectory = PositionModelPoseTrajectory(robotWaypoints,'R3','smoothingspline');
primitive1Trajectory = ConstantMotionDiscretePoseTrajectory(t,primitive1InitialPose_R3xso3,primitive1Motion_R3xso3,'R3xso3');
primitive2Trajectory = ConstantMotionDiscretePoseTrajectory(t,primitive2InitialPose_R3xso3,primitive2Motion_R3xso3,'R3xso3');
constantSE3Object1Motion = primitive1Trajectory.RelativePoseGlobalFrameSE3(t(1),t(2));
constantSE3Object2Motion = primitive2Trajectory.RelativePoseGlobalFrameSE3(t(1),t(2));
constantSE3ObjectMotion = [[constantSE3Object1Motion(1:3,4);...
    arot(constantSE3Object1Motion(1:3,1:3))],...
    [constantSE3Object2Motion(1:3,4);arot(constantSE3Object2Motion(1:3,1:3))]];

environment = Environment();
environment.addEllipsoid([0.5 0.5 0.8],8,'R3',primitive1Trajectory);
environment.addEllipsoid([0.5 0.5 0.8],8,'R3',primitive2Trajectory);

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
axisLimits = [-30,30,-5,30,-2,2];
% title('Environment')
axis equal
xlabel('x')
ylabel('y')
zlabel('z')
view(viewPoint)
axis(axisLimits)
hold on
grid on
primitive1Trajectory.plot(t,[0 0 0],'axesOFF')
primitive2Trajectory.plot(t,[0 0 0],'axesOFF')
cameraTrajectory.plot(t,[0 0 1],'axesOFF')
frames = sensor.plot(t,environment);
% implay(frames);

    %% 4.a output video
% v = VideoWriter('Data/Videos/App6_sensor_environment.mp4','MPEG-4');
% open(v)
% writeVideo(v,frames);
% close(v)

%% 5. Generate Measurements & Save to Graph File
sensor.generateMeasurements(config);
config.set('constantSE3Motion',constantSE3ObjectMotion);
writeDataAssociationVerticesEdges(config,constantSE3ObjectMotion);

%% 6. load graph files
groundTruthCell  = graphFileToCell(config,config.groundTruthFileName);
measurementsCell = graphFileToCell(config,config.measurementsFileName);

%% 7. Manually recreate vertexes
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
graphN.saveGraphFile(config,'app6_results.graph');

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
resultsCell = graphFileToCell(config,'app6_results.graph');
plotGraphFile(config,resultsCell,[1 0 0])