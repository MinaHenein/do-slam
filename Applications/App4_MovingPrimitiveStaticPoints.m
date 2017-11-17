%--------------------------------------------------------------------------
% Author: Yash Vyas - yjvyas@gmail.com - 26/08/2017
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
config.set('groundTruthFileName','app4_groundTruth.graph');
config.set('measurementsFileName','app4_measurements.graph');

%% 2. Generate Environment
if config.rngSeed
    rng(config.rngSeed); 
end

robotWaypoints = [0:2:tN; 0 10 15 20 15 10; 0 0 5 10 15 15; 0 0 1 2 4 2];
primitiveWaypoints = [0:2:tN; 10 20 25 20 10 5; 0 0 10 15 15 15; 0 2 1 3 5 2];

% construct trajectories
robotTrajectory = PositionModelPoseTrajectory(robotWaypoints,'R3','smoothingspline');
primitiveTrajectory = PositionModelPoseTrajectory(primitiveWaypoints,'R3','smoothingspline');

environment = Environment();
environment.addEllipsoid([1 1 2],12,'R3',primitiveTrajectory);
environment.addStaticPoints([30*ones(1,50); 20*rand(2,50)]);
environment.addStaticPoints([20*rand(1,50); 20*ones(1,50); 20*rand(1,50)]);

%% 3. Initialise Sensor
cameraTrajectory = RelativePoseTrajectory(robotTrajectory,config.cameraRelativePose);

% occlusion sensor
sensor = SimulatedEnvironmentOcclusionSensor();
sensor.addEnvironment(environment);
sensor.addCamera(config.fieldOfView,cameraTrajectory);
sensor.setVisibility(config,environment);

% figure
% spy(sensor.get('pointVisibility'));

%% 4. Plot Environment
% figure
% hold on
% grid on
% axis equal
% viewPoint = [-50,25];
% axisLimits = [-5,30,-5,20,-5,20];
% axis equal
% xlabel('x')
% ylabel('y')
% zlabel('z')
% view(viewPoint)
% axis(axisLimits)
% primitiveTrajectory.plot(t,[0 0 0],'axesOFF')
% cameraTrajectory.plot(t,[0 1 1],'axesOFF')
% frames = sensor.plot(t(1),environment);
% % implay(frames);

    %% 4.a output video
    % add code here for that
    
%% 5. Generate Measurements & Save to Graph File
sensor.generateMeasurements(config);

%% 6. load graph files
groundTruthCell  = graphFileToCell(config,config.groundTruthFileName);
measurementsCell = graphFileToCell(config,config.measurementsFileName);

%% 7. Manual recreation of vertices
% [initialCell1, initialCell2] = recreateInitialVertexes(config,measurementsCell,groundTruthCell);

% initialGraph1 = Graph().graphFileToGraph(config,initialCell1);
% initialGraph2 = Graph().graphFileToGraph(config,initialCell2);
% initialGraph1.saveGraphFile(config,'app4_initial1.graph');
% initialGraph2.saveGraphFile(config,'app4_initial2.graph');

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
%save results to graph file
graphN.saveGraphFile(config,'app4_results.graph');

%% 9. Error analysis
%load ground truth into graph, sort if required
graphGT = Graph(config,groundTruthCell);
% fprintf('Initial 1 error: \n');
% resultsInitial1 = errorAnalysis(config,graphGT,initialGraph1);
% fprintf('Initial 2 error: \n');
% resultsInitial2 = errorAnalysis(config,graphGT,initialGraph2);
% fprintf('Results error: \n');
results = errorAnalysis(config,graphGT,graphN);

%% 10. Plot
    %% 10.1 Plot intial, final and ground-truth solutions
%no constraints
% figure
% subplot(1,2,1)
% spy(solverEnd.systems(end).A)
% subplot(1,2,2)
% spy(solverEnd.systems(end).H)

h = figure; 
xlabel('x')
ylabel('y')
zlabel('z')
hold on
grid on
view([-50,25])
%plot groundtruth
plotGraphFile(config,groundTruthCell,[0 0 1]);
%plot results
resultsCell = graphFileToCell(config,'app4_results.graph');
plotGraphFile(config,resultsCell,[1 0 0])