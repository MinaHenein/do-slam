%--------------------------------------------------------------------------
% Author: Mina Henein - mina.henein@anu.edu.au - 30/11/2018
% Contributors:
%--------------------------------------------------------------------------
% Test static only reprojectioin error minimization
%--------------------------------------------------------------------------
% vKitti main
config = CameraConfig();
K = [725,   0,     620.5;
    0,    725,     187.0;
    0,      0,        1];
config.set('focalLength',725);
config.set('opticalCentreX',620.5);
config.set('opticalCentreY',187.0);
config = setAppConfig(config);
config.absoluteToRelativePointHandle = @AbsoluteToRelativePositionR3xso3Image;
config.relativeToAbsolutePointHandle = @RelativeToAbsolutePositionR3xso3Image; 
config.set('landmarkErrorToMinimize' ,'reprojectionKnownIntrinsics');

%% 5. Generate Measurements & Save to Graph File, load graph file as well
config.set('pointMotionMeasurement','Off')
config.set('measurementsFileName','finalNoiseSequence0001_334to426_final_MeasStaticOnly.graph')
config.set('groundTruthFileName','finalNoiseSequence0001_334to426_final_GTStaticOnly.graph')
groundTruthCellStaticOnly = graphFileToCell(config,config.groundTruthFileName);
measurementsCellStaticOnly = graphFileToCell(config,config.measurementsFileName);
 
%% 6. Solve
%% 6.1 Without SE3
timeStart = tic;
initialGraph0 = Graph();
initialSolver = initialGraph0.process(config,measurementsCellStaticOnly,groundTruthCellStaticOnly);
initialSolverEnd = initialSolver(end);
totalTime = toc(timeStart);
fprintf('\nTotal time solving: %f\n',totalTime)
%get desired graphs & systems
initialGraph0  = initialSolverEnd.graphs(1);
initialGraphN  = initialSolverEnd.graphs(end);
%save results to graph file
initialGraphN.saveGraphFile(config,'finalNoiseSequence0001_334to426_final_resultsStaticOnly.graph');

%% 7. Error analysis
% load ground truth into graph, sort if required
graphGTNoSE3 = Graph(config,groundTruthCellStaticOnly);
fprintf('\nInitial results for without SE(3) Transform:\n')
resultsNoSE3 = errorAnalysis(config,graphGTNoSE3,initialGraphN);

%% 8. Plot
figure('units','normalized','color','w');
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
% plotGraphFileICRA(config,groundTruthCellStaticOnly,'groundTruth');
%plot results
resultsNoSE3Cell = graphFileToCell(config,'finalNoiseSequence0001_334to426_final_resultsStaticOnly.graph');
plotGraphFileICRA(config,resultsNoSE3Cell,'initial',...
    resultsNoSE3.relPose.get('R3xso3Pose'),resultsNoSE3.posePointsN.get('R3xso3Pose'))