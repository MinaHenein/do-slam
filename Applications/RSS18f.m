%--------------------------------------------------------------------------
% Author: Mina Henein - mina.henein@anu.edu.au - 17/01/18
% Contributors:
%--------------------------------------------------------------------------
function RSS18f(factor, noiseLevel)
%% 1. Config
% time
t0 = 1;
tN = 24*factor;
nSteps = 24*factor;
t = linspace(t0,tN,nSteps);
dt = (tN-t0)/(nSteps);

if factor < 20
    a = dt;
elseif factor >= 20 && factor <= 50 
    a = dt/10;
else
    a = dt/20;
end

config = CameraConfig();
config = setAppConfig(config);
if noiseLevel == 1  
    %config = setLowErrorAppConfig(config);
    config.set('std2PointsSE3Motion', [0.01,0.01,0.01]');
elseif noiseLevel == 2
    %config = setAppConfig(config);
    config.set('std2PointsSE3Motion', [0.05,0.05,0.05]');
elseif noiseLevel == 3
    %config = setHighErrorAppConfig(config);
    config.set('std2PointsSE3Motion', [0.1,0.1,0.1]');
end
% config.set('noiseModel','Off');
config.set('groundTruthFileName',strcat('RSS18f2_groundTruth_',num2str(factor),...
    '_',num2str(noiseLevel),'.graph'));
config.set('measurementsFileName',strcat('RSS18f2_measurements_',num2str(factor),...
    '_',num2str(noiseLevel),'.graph'));
config.set('t',t);

% SE3 Motion
config.set('pointMotionMeasurement','point2DataAssociation');
config.set('motionModel','constantSE3MotionDA');
%% 2. Generate Environment
if config.rngSeed
    rng(config.rngSeed);
end

%% 2.1 robot waypoints and trajectory
t1 = linspace(t0,tN/2,nSteps/2);
t2 = linspace(tN/2,t0,nSteps/2);
% shiftting spiral down
robotWaypoints1 = [sin(t1 * .5); cos(t1 * .5) + t1*.1 ; (25 - 10 * (.5 + .5 * sin(t1 /10)))]';
% shiftting spiral up
robotWaypoints2 = [-sin(t2 * .5); cos(t2 * .5) + t2*.1; (25 - 10 * (.5 + .5 * sin(t2/ 10)))]';
robotWaypoints = [robotWaypoints1; robotWaypoints2];
robotWaypoints = reshape(robotWaypoints',[size(robotWaypoints,2),size(robotWaypoints,1)]);
robotTrajectoryWaypoints = [t;robotWaypoints];
robotTrajectory = PositionModelPoseTrajectory(robotTrajectoryWaypoints,'R3','smoothingspline');
%% 2.2 objects waypoints and trajectories
% primitive 1
primitive1InitialPose_R3xso3 = [-2 5 -2 pi/3 0 pi/2]';
primitive1Motion_R3xso3 = [0.105*dt; 0; 0; 0.005*arot(eul2rot([0.105*dt,0,0]))];
primitive1Trajectory = ConstantMotionDiscretePoseTrajectory(t,...
    primitive1InitialPose_R3xso3,primitive1Motion_R3xso3,'R3xso3');
% primitive 2
primitive2InitialPose_R3xso3 = [-5 2 -2 pi/3 0 pi/2]';
primitive2Motion_R3xso3 = -primitive1Motion_R3xso3;
primitive2Trajectory = ConstantMotionDiscretePoseTrajectory(t,...
    primitive2InitialPose_R3xso3,primitive2Motion_R3xso3,'R3xso3');
% primitive 3
primitive3InitialPose_R3xso3 = [2 5 -2 pi/3 0 -pi/2]';
primitive3Motion_R3xso3 = [-0.21*dt; 0; 0; 0.005*arot(eul2rot([-0.105*dt,0,0]))];
primitive3Trajectory = ConstantMotionDiscretePoseTrajectory(t,...
    primitive3InitialPose_R3xso3,primitive3Motion_R3xso3,'R3xso3');
% primitive 4
primitive4InitialPose_R3xso3 = [10 1 -2 pi/3 0 -pi/2]';
primitive4Motion_R3xso3 = -primitive3Motion_R3xso3;
primitive4Trajectory = ConstantMotionDiscretePoseTrajectory(t,...
    primitive4InitialPose_R3xso3,primitive4Motion_R3xso3,'R3xso3');
% primitive 5
primitive5InitialPose_R3xso3 = [0 18 0 pi/6 0 -pi/3]';
primitive5Motion_R3xso3 = a*[1.08; 1.21; 0; arot(eul2rot([0,0,0]))];
primitive5Trajectory = ConstantMotionDiscretePoseTrajectory(t,...
    primitive5InitialPose_R3xso3,primitive5Motion_R3xso3,'R3xso3');
% line1 = [linspace(0,25,nSteps);linspace(18,46,nSteps);zeros(1,nSteps)];
% primitive5Waypoints = [t; line1];
% primitive5Trajectory = PositionModelPoseTrajectory(primitive5Waypoints,'R3','linearinterp');
% primitive 6
primitive6InitialPose_R3xso3 = [40 25 0 pi/6 0 -pi/3]';
primitive6Motion_R3xso3 = a*[-0.86; 0.21; 0; arot(eul2rot([0,0,0]))];
primitive6Trajectory = ConstantMotionDiscretePoseTrajectory(t,...
    primitive6InitialPose_R3xso3,primitive6Motion_R3xso3,'R3xso3');
% line2 = [linspace(40,20,nSteps);linspace(25,30,nSteps);zeros(1,nSteps)];
% primitive6Waypoints = [t; line2];
% primitive6Trajectory = PositionModelPoseTrajectory(primitive6Waypoints,'R3','linearinterp');
% primitive 7
primitive7InitialPose_R3xso3 = [20 20 10 pi/3 0 -pi/6]';
primitive7Motion_R3xso3 = a*[-0.17; 1.08; 0; arot(eul2rot([0,0,0]))];
primitive7Trajectory = ConstantMotionDiscretePoseTrajectory(t,...
    primitive7InitialPose_R3xso3,primitive7Motion_R3xso3,'R3xso3');
% line3 = [linspace(20,16,nSteps);linspace(20,45,nSteps);linspace(10,10,nSteps)];
% primitive7Waypoints = [t; line3];
% primitive7Trajectory = PositionModelPoseTrajectory(primitive7Waypoints,'R3','linearinterp');
% primitive 8
primitive8InitialPose_R3xso3 = [10 -15 0 pi/3 0 -pi/6]';
primitive8Motion_R3xso3 = a*[-0.43; -0.86; 0.43; arot(eul2rot([0,0,0]))];
primitive8Trajectory = ConstantMotionDiscretePoseTrajectory(t,...
    primitive8InitialPose_R3xso3,primitive8Motion_R3xso3,'R3xso3');
% line4 = [linspace(10,0,nSteps);linspace(-15,-35,nSteps);linspace(0,10,nSteps)];
% primitive8Waypoints = [t; line4];
% primitive8Trajectory = PositionModelPoseTrajectory(primitive8Waypoints,'R3','linearinterp');
%% constant SE3 motion
constantSE3ObjectMotion = [];
constantSE3ObjectMotion(:,1) = primitive1Trajectory.RelativePoseGlobalFrameR3xso3(t(1),t(2));
constantSE3ObjectMotion(:,2) = primitive2Trajectory.RelativePoseGlobalFrameR3xso3(t(1),t(2));
constantSE3ObjectMotion(:,3) = primitive3Trajectory.RelativePoseGlobalFrameR3xso3(t(1),t(2));
constantSE3ObjectMotion(:,4) = primitive4Trajectory.RelativePoseGlobalFrameR3xso3(t(1),t(2));
constantSE3ObjectMotion(:,5) = primitive5Trajectory.RelativePoseGlobalFrameR3xso3(t(1),t(2));
constantSE3ObjectMotion(:,6) = primitive6Trajectory.RelativePoseGlobalFrameR3xso3(t(1),t(2));
constantSE3ObjectMotion(:,7) = primitive7Trajectory.RelativePoseGlobalFrameR3xso3(t(1),t(2));
constantSE3ObjectMotion(:,8) = primitive8Trajectory.RelativePoseGlobalFrameR3xso3(t(1),t(2));
%% construct environment
environment = Environment();
environment.addEllipsoid([1 1 2.5],8,'R3',primitive1Trajectory);
environment.addEllipsoid([1 1 2.5],8,'R3',primitive2Trajectory);
environment.addEllipsoid([1 1 2.5],8,'R3',primitive3Trajectory);
environment.addEllipsoid([1 1 2.5],8,'R3',primitive4Trajectory);
environment.addEllipsoid([1 1 2.5],8,'R3',primitive5Trajectory);
environment.addEllipsoid([1 1 2.5],8,'R3',primitive6Trajectory);
environment.addEllipsoid([1 1 2.5],8,'R3',primitive7Trajectory);
environment.addEllipsoid([1 1 2.5],8,'R3',primitive8Trajectory);
%% 3. Initialise Sensor
cameraTrajectory = RelativePoseTrajectory(robotTrajectory,config.cameraRelativePose);
% occlusion sensor
sensor = SimulatedEnvironmentOcclusionSensor();
sensor.addEnvironment(environment);
sensor.addCamera(config.fieldOfView,cameraTrajectory);
sensor.setVisibility(config,environment);

figure
spy(sensor.get('pointVisibility'));
%% 4. Plot Environment
% figure
% viewPoint = [-35,35];
% % axisLimits = [-30,30,-5,30,-10,10];
% % title('Environment')
% axis equal
% xlabel('x (m)')
% ylabel('y (m)')
% zlabel('z (m)')
% view(viewPoint)
% % axis(axisLimits)
% hold on
% grid on
% primitive1Trajectory.plot(t,[0 0 0],'axesOFF')
% primitive2Trajectory.plot(t,[0 0 0],'axesOFF')
% primitive3Trajectory.plot(t,[0 0 0],'axesOFF')
% primitive4Trajectory.plot(t,[0 0 0],'axesOFF')
% primitive5Trajectory.plot(t,[0 0 0],'axesOFF')
% primitive6Trajectory.plot(t,[0 0 0],'axesOFF')
% primitive7Trajectory.plot(t,[0 0 0],'axesOFF')
% primitive8Trajectory.plot(t,[0 0 0],'axesOFF')
% cameraTrajectory.plot(t,[0 0 1],'axesOFF')
% % set(gcf,'Position',[0 0 1024 768]);
% frames = sensor.plot(t,environment);
% implay(frames);

%% 4.a output video
% v = VideoWriter('Data/Videos/App6_sensor_environment.mp4','MPEG-4');
% open(v)
% writeVideo(v,frames);
% close(v)

%% 5. Generate Measurements & Save to Graph File, load graph file as well
config.set('constantSE3Motion',constantSE3ObjectMotion);
%% 5.1 For initial (without SE3)
config.set('pointMotionMeasurement','Off')
config.set('measurementsFileName',strcat('RSS18f2_measurementsNoSE3_',num2str(factor),'_',...
    num2str(noiseLevel),'.graph'))
config.set('groundTruthFileName',strcat('RSS18f2_groundTruthNoSE3_',num2str(factor),'_',...
    num2str(noiseLevel),'.graph'))
sensor.generateMeasurements(config);
groundTruthNoSE3Cell = graphFileToCell(config,config.groundTruthFileName);
measurementsNoSE3Cell = graphFileToCell(config,config.measurementsFileName);

%% 5.2 For test (with SE3)
config.set('pointMotionMeasurement','point2DataAssociation');
config.set('measurementsFileName',strcat('RSS18f2_measurements_',num2str(factor),'_',...
    num2str(noiseLevel),'.graph'));
config.set('groundTruthFileName',strcat('RSS18f2_groundTruth_',num2str(factor),'_',...
    num2str(noiseLevel),'.graph'));
sensor.generateMeasurements(config);
writeDataAssociationVerticesEdges_constantSE3Motion(config,constantSE3ObjectMotion);
measurementsCell = graphFileToCell(config,config.measurementsFileName);
groundTruthCell  = graphFileToCell(config,config.groundTruthFileName);

%% 6. Solve
% config.set('sortVertices',1);
% config.set('sortEdges',1);
%% 6.1 Without SE3
timeStart = tic;
initialGraph0 = Graph();
initialSolver = initialGraph0.process(config,measurementsNoSE3Cell,groundTruthNoSE3Cell);
initialSolverEnd = initialSolver(end);
totalTime = toc(timeStart);
fprintf('\nTotal time solving: %f\n',totalTime)

%get desired graphs & systems
initialGraph0  = initialSolverEnd.graphs(1);
initialGraphN  = initialSolverEnd.graphs(end);
%save results to graph file
initialGraphN.saveGraphFile(config,strcat('RSS18f2_resultsNoSE3_',num2str(factor),'_',...
    num2str(noiseLevel),'.graph'));

%% 6.2 With SE3
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
graphN.saveGraphFile(config,strcat('RSS18f2_results_',num2str(factor),'_',...
    num2str(noiseLevel),'.graph'));

%% 7. Error analysis
%load ground truth into graph, sort if required
graphGT = Graph(config,groundTruthCell);
graphGTNoSE3 = Graph(config,groundTruthNoSE3Cell);
fprintf('\nInitial results for without SE(3) Transform:\n')
resultsNoSE3 = errorAnalysis(config,graphGTNoSE3,initialGraphN);
fprintf('\nFinal results for SE(3) Transform:\n')
resultsSE3 = errorAnalysis(config,graphGT,graphN);

% save results
H = solverEnd.systems(end).H;
save(strcat('Data/RSS18f_results/results2NoSE3_',num2str(factor),'_',num2str(noiseLevel)),'resultsNoSE3')
save(strcat('Data/RSS18f_results/results2SE3_',num2str(factor),'_',num2str(noiseLevel)),'resultsSE3')
save(strcat('Data/RSS18f_results/H2_',num2str(factor),'_',num2str(noiseLevel)),'H')

%% 8. Plot
%% 8.1 Plot initial, final and ground-truth solutions
%no constraints
figure
spy(solverEnd.systems(end).H)

% figure
% spy(chol(solverEnd.systems(end).H))

% h = figure;
% xlabel('x (m)')
% ylabel('y (m)')
% zlabel('z (m)')
% hold on
% grid on
% axis equal
% axisLimits = [-25,25,0,30,-5,15];
% axis(axisLimits)
% view([-50,25])
% %plot groundtruth
% plotGraphFileICRA(config,groundTruthCell,'groundTruth');
% %plot results
% resultsNoSE3Cell = graphFileToCell(config,'RSS18f_resultsNoSE3.graph');
% resultsCell = graphFileToCell(config,'RSS18f_results.graph');
% plotGraphFileICRA(config,resultsNoSE3Cell,'initial',resultsNoSE3.relPose.get('R3xso3Pose'),...
%     resultsNoSE3.posePointsN.get('R3xso3Pose'))
% plotGraphFileICRA(config,resultsCell,'solverResults',resultsSE3.relPose.get('R3xso3Pose'),...
%     resultsSE3.posePointsN.get('R3xso3Pose'))

end