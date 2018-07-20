%--------------------------------------------------------------------------
% Author: Mina Henein - mina.henein@anu.edu.au - 20/05/2018
% Contributors:
%--------------------------------------------------------------------------
%vKitti static only
%% 1. Config
config = CameraConfig();
config = setAppConfig(config);

%% 5. Generate Measurements & Save to Graph File, load graph file as well
%% 5.1 without SE3
config.set('pointMotionMeasurement','Off')
% config.set('measurementsFileName','staticOnlyNoiseMeas2.graph')
% config.set('groundTruthFileName','staticOnlyNoiseGT2.graph')
config.set('measurementsFileName','staticDynamic92ImagesStaticOnlyMeasTest.graph')
config.set('groundTruthFileName','staticDynamic92ImagesStaticOnlyGTTest.graph')
% config.set('measurementsFileName','vKitti_Meas_staticOnlyTest.graph')
% config.set('groundTruthFileName','vKitti_GT_staticOnlyTest.graph')
groundTruthNoSE3Cell = graphFileToCell(config,config.groundTruthFileName);
measurementsNoSE3Cell = graphFileToCell(config,config.measurementsFileName);

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
initialGraphN.saveGraphFile(config,'staticOnly92ImagesVKITTI.graph');

%% 7. Error analysis
%load ground truth into graph, sort if required
graphGTNoSE3 = Graph(config,groundTruthNoSE3Cell);
fprintf('\nInitial results for without SE(3) Transform:\n')
resultsNoSE3 = errorAnalysis(config,graphGTNoSE3,initialGraphN);