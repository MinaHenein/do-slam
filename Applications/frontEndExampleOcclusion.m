%--------------------------------------------------------------------------
% Author: Yash Vyas - yjvyas@gmail.com, 30/05/2017 
%         Montiel Abello - montiel.abello@gmail.com - 23/05/17
% Contributors:
%--------------------------------------------------------------------------

clear all 
close all 

%% 1. Config
% time
nSteps = 51;
t0 = 0;
tN = 10;
t  = linspace(t0,tN,nSteps);

% CameraConfig is subclass of Config with properties specific to
% applications with a camera
config = CameraConfig();
config = setAppConfig(config);
% set properties of Config
config.set('t',t);
config.set('rngSeed',1);
config.set('noiseModel','Gaussian');
% config.set('noiseModel','Off');
config.set('processing','batch');
config.set('poseParameterisation','R3xso3');
% config.set('poseParameterisation','logSE3');
config.set('poseVertexLabel'     ,'VERTEX_POSE_LOG_SE3');
config.set('pointVertexLabel'    ,'VERTEX_POINT_3D');
config.set('planeVertexLabel'    ,'VERTEX_PLANE_4D');
config.set('posePoseEdgeLabel'   ,'EDGE_LOG_SE3');
config.set('posePointEdgeLabel'  ,'EDGE_3D');
config.set('pointPlaneEdgeLabel' ,'EDGE_1D');
config.set('graphFileFolderName' ,'OcclusionTesting');
config.set('groundTruthFileName' ,'groundTruth.graph');
config.set('measurementsFileName','measurements.graph');
config.set('stdPosePrior' ,[0.001,0.001,0.001,pi/600,pi/600,pi/600]');
config.set('stdPointPrior',[0.001,0.001,0.001]');
config.set('stdPosePose'  ,[0.01,0.01,0.01,pi/90,pi/90,pi/90]');
config.set('stdPosePoint' ,[0.02,0.02,0.02]');
config.set('stdPointPlane',0.001);
% set properties of CameraConfig
config.set('fieldOfView',[-pi/3,pi/3,-pi/6,pi/6,1,10]); %az,el,r limits
config.set('cameraRelativePose',GP_Pose([0,0,0,0,0,-pi/8]'));

%% 2. Generate Environment
% waypoints
staticPose1  = [5,8,0,0,0,0]';
staticPose2  = [4 16 4 pi/2 0 0]';
dynamicWaypoints = [0:2:tN; 4 5 7 10 7 3; 12 11 10 5 8 14; 8 5 2 3 6 4];

% construct trajectories
staticTrajectory1 = StaticPoseTrajectory(staticPose1,'R3xso3');
staticTrajectory2 = StaticPoseTrajectory(staticPose2,'R3xso3');
dynamicTrajectory = PositionModelPoseTrajectory(dynamicWaypoints,'R3','smoothingspline');

if config.rngSeed 
    rng(config.rngSeed); 
end

environment = Environment();
environment.addRectangle([10,15],100,'mixed',staticTrajectory1);
environment.addRectangle([8,6],50,'mixed',staticTrajectory2);
% environment.addEllipsoid([1 1 2],10,'R3',dynamicTrajectory);
environment.addCube(1, 'R3',dynamicTrajectory);

%% 3. Plot
% figure
% viewPoint = [-50,25];
% axisLimits = [-1,15,-1,20,-1,10];
% title('Environment')
% axis equal
% xlabel('x')
% ylabel('y')
% zlabel('z')
% view(viewPoint)
% axis(axisLimits)
% hold on
% staticTrajectory1.plot()
% staticTrajectory2.plot()
% dynamicTrajectory.plot(t)
% environment.plot(t)

%% 3. Initialise Sensor
robotTrajectory   = PositionModelPoseTrajectory(dynamicWaypoints,'R3','smoothingspline');
cameraTrajectory = RelativePoseTrajectory(robotTrajectory,config.cameraRelativePose);
sensor = SimulatedEnvironmentOcclusionSensor();
sensor.addEnvironment(environment);
sensor.addCamera(config.fieldOfView,cameraTrajectory);
sensor.setVisibility(config,environment); % don't have any way of testing this as of now
% 
%% 5. Generate Measurements & Save to Graph File
sensor.generateMeasurements(config);

%% 6. Plot
figure
viewPoint = [-50,25];
axisLimits = [-1,15,-1,20,-1,10];
title('Environment')
axis equal
xlabel('x')
ylabel('y')
zlabel('z')
view(viewPoint)
axis(axisLimits)
hold on
staticTrajectory1.plot()
staticTrajectory2.plot()
cameraTrajectory.plot(t)
environment.plot([3])

%% 6. load graph files
groundTruthCell  = graphFileToCell(config,config.groundTruthFileName);
measurementsCell = graphFileToCell(config,config.measurementsFileName);

%% 7. Solve
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
graphN.saveGraphFile(config,'resultsFrontEnd.graph');

%% 8. Error analysis
%load ground truth into graph, sort if required
graphGT = Graph(config,groundTruthCell);
results = errorAnalysis(config,graphGT,graphN);

%% 9. Plot
    %% 9.1 Plot environment
figure
viewPoint = [-50,25];
axisLimits = [-1,15,-1,20,-1,10];
title('Environment')
axis equal
xlabel('x')
ylabel('y')
zlabel('z')
view(viewPoint)
axis(axisLimits)
hold on
staticTrajectory1.plot()
staticTrajectory2.plot()
cameraTrajectory.plot(t)
environment.plot(t)
    %% 9.1 Plot intial, final and ground-truth solutions
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
%plot initial solution
plotGraph(config,graph0,[0 1 1]);
%plot groundtruth
plotGraphFile(config,groundTruthCell,[1 0 0]);
%plot results
resultsCell = graphFileToCell(config,'results.graph');
plotGraphFile(config,resultsCell,'r')