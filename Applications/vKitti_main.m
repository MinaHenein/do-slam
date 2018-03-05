%--------------------------------------------------------------------------
% Author: Mina Henein - mina.henein@anu.edu.au - 01/02/2018
% Contributors:
%--------------------------------------------------------------------------
%vKitti main
%% 1. Config
% objectPoses = load('objPose_test2.mat');
% objectPoses = objectPoses.objPose;
% 
% nObjects = 3;
% constantSE3ObjectMotion = [];
% 
% for i=1:nObjects
%     poses = objectPoses(i).pose;
%     rotations= {};
%     translations = [];
%     for j=2:size(poses,1)
%         objectMotion = AbsoluteToRelativePoseR3xso3(poses(j-1,:)', poses(j,:)');
%         objectMotion = poseToTransformationMatrix(objectMotion);
%         rotM = objectMotion(1:3,1:3);
%         t = objectMotion(1:3,4);
%         rotations{j-1} = rotM;
%         translations(:,j-1) = t;
%     end
%     R = rotationAveraging(rotations);
%     t = mean(translations,2);
%     SE3Motion = [t;arot(R)];
%     constantSE3ObjectMotion(:,i) =SE3Motion;
% end

config = CameraConfig();
config = setAppConfig(config);
config.set('motionModel','constantSE3MotionDA');
config.set('std2PointsSE3Motion', [0.05,0.05,0.05]');
config.set('SE3MotionVertexInitialization','eye');

%% 5. Generate Measurements & Save to Graph File, load graph file as well

% config.set('constantSE3Motion',constantSE3ObjectMotion);
%% 5.1 For initial (without SE3)
config.set('pointMotionMeasurement','Off')
config.set('measurementsFileName','vKitti_measurementsNoSE3_test2.graph')
config.set('groundTruthFileName','vKitti_groundTruthNoSE3_test2.graph')
groundTruthNoSE3Cell = graphFileToCell(config,config.groundTruthFileName);
measurementsNoSE3Cell = graphFileToCell(config,config.measurementsFileName);

%% 5.2 For test (with SE3)
config.set('pointMotionMeasurement','point2DataAssociation');
config.set('measurementsFileName','vKitti_measurements_test2.graph');
config.set('groundTruthFileName','vKitti_groundTruth_test2.graph');
writeDataAssociationVerticesEdges_constantSE3MotionNoGT(config);
measurementsCell = graphFileToCell(config,config.measurementsFileName);
groundTruthCell  = graphFileToCell(config,config.groundTruthFileName);

%% 6. Solve
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
initialGraphN.saveGraphFile(config,'vKitti_resultsNoSE3.graph');


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
graphN.saveGraphFile(config,'vKitti_results.graph');

%% 7. Error analysis
%load ground truth into graph, sort if required
graphGTNoSE3 = Graph(config,groundTruthNoSE3Cell);
graphGT = Graph(config,groundTruthCell);
fprintf('\nInitial results for without SE(3) Transform:\n')
resultsNoSE3 = errorAnalysis(config,graphGTNoSE3,initialGraphN);
fprintf('\nFinal results for SE(3) Transform:\n')
resultsSE3 = errorAnalysis(config,graphGT,graphN);

%% 8. Plot
%% 8.1 Plot initial, final and ground-truth solutions
%no constraints
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
% axisLimits = [-30,50,-10,60,-25,25];
% axis(axisLimits)
view([-50,25])
%plot groundtruth
plotGraphFileICRA(config,groundTruthCell,'groundTruth');
%plot results
resultsNoSE3Cell = graphFileToCell(config,'vKitti_resultsNoSE3.graph');
resultsCell = graphFileToCell(config,'vKitti_results.graph');
plotGraphFileICRA(config,resultsNoSE3Cell,'initial',resultsNoSE3.relPose.get('R3xso3Pose'),resultsNoSE3.posePointsN.get('R3xso3Pose'))
plotGraphFileICRA(config,resultsCell,'solverResults',resultsSE3.relPose.get('R3xso3Pose'),resultsSE3.posePointsN.get('R3xso3Pose'))