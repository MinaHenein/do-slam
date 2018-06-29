%--------------------------------------------------------------------------
% Author: Mina Henein - mina.henein@anu.edu.au - 28/06/2018
% To test App_7 static only points, pose error results should be the same 
% as App_7 No SE3 results 
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
config = setAppConfig(config);
config.set('t',t);
config.set('staticDataAssociation','On');

%% 2. Generate Environment
if config.rngSeed
    rng(config.rngSeed); 
end

primitiveInitialPose_R3xso3 = [10 3 3 0 0 0.2]';
primitiveMotion_R3xso3 = [0.8*dt; 0; 0; arot(eul2rot([0.06*dt,0,0.001*dt]))];

% construct trajectories
primitiveTrajectory = ConstantMotionDiscretePoseTrajectory(t,primitiveInitialPose_R3xso3,primitiveMotion_R3xso3,'R3xso3');

sampleTimes = t(1:floor(numel(t)/5):numel(t));
sampleWaypoints = primitiveTrajectory.get('R3xso3Pose',sampleTimes);
robotWaypoints = [linspace(0,tN+5,7); 0 sampleWaypoints(1,:); 0 (sampleWaypoints(2,:)+0.1); 0 (sampleWaypoints(3,:)-0.1)];
robotTrajectory = PositionModelPoseTrajectory(robotWaypoints,'R3','smoothingspline');

environment = Environment();
nPoints = 50;
environment.addStaticPoints([30*ones(1,nPoints); 40*rand(1,nPoints); 20*rand(1,nPoints)]);
environment.addStaticPoints([30*rand(1,nPoints); 40*ones(1,nPoints); 20*rand(1,nPoints)]);

%% 3. Initialise Sensor
cameraTrajectory = RelativePoseTrajectory(robotTrajectory,config.cameraRelativePose);
% occlusion sensor
sensor = SimulatedEnvironmentOcclusionSensor();
sensor.addEnvironment(environment);
sensor.addCamera(config.fieldOfView,cameraTrajectory);
sensor.setVisibility(config,environment);

figure
spy(sensor.get('pointVisibility'));

%% 5. Generate Measurements & Save to Graph File, load graph file as well
    %% 5.1 For initial (without SE3)
    config.set('pointMotionMeasurement','Off');
    config.set('groundTruthFileName','app7_groundTruthStaticOnly.graph');
    config.set('measurementsFileName','app7_measurementsStaticOnly.graph');
    sensor.generateMeasurements(config);
    measurementsStaticOnlyCell = graphFileToCell(config,config.measurementsFileName);
    groundTruthStaticOnlyCell  = graphFileToCell(config,config.groundTruthFileName);
   
%% 6. Solve
    %% 6.1 Without SE3
    %no constraints
    timeStart = tic;
    initialGraph0 = Graph();
    initialSolver = initialGraph0.process(config,measurementsStaticOnlyCell,groundTruthStaticOnlyCell);
    initialSolverEnd = initialSolver(end);
    totalTime = toc(timeStart);
    fprintf('\nTotal time solving: %f\n',totalTime)

    %get desired graphs & systems
    initialGraph0  = initialSolverEnd.graphs(1);
    initialGraphN  = initialSolverEnd.graphs(end);
    %save results to graph file
    initialGraphN.saveGraphFile(config,'app7_resultsStaticOnly.graph');
    
%% 7. Error analysis
%load ground truth into graph, sort if required
graphGTStaticOnly = Graph(config,groundTruthStaticOnlyCell);

fprintf('\nInitial results for without SE(3) Transform:\n')
resultsStaticOnly = errorAnalysis(config,graphGTStaticOnly,initialGraphN);