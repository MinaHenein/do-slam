%--------------------------------------------------------------------------
% Author: Mina Henein - mina.henein@anu.edu.au - 28/08/2018
% Contributors:
%--------------------------------------------------------------------------
% vKitti posesOnly
%% Config
config = CameraConfig();
config = setAppConfig(config);
config.set('processing','batch');
config.set('measurementsFileName','finalNoiseSequence0001_MeasPosesOnly.graph');
config.set('groundTruthFileName','finalNoiseSequence0001_GTPosesOnly.graph'); 
measurementsCell = graphFileToCell(config,config.measurementsFileName);
groundTruthCell  = graphFileToCell(config,config.groundTruthFileName);

%% Solve
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
graphN.saveGraphFile(config,strcat(config.groundTruthFileName(1:end-12),'results.graph'));

%% Error analysis
graphGT = Graph(config,groundTruthCell);
fprintf('\nFinal results for SE(3) Transform:\n')
results = errorAnalysis(config,graphGT,graphN);

%% Plot
figure('units','normalized','color','w');
xlabel('x (m)')
ylabel('y (m)')
zlabel('z (m)')
hold on
grid on
axis equal
view([-50,25])
%plot groundtruth
plotGraphFileICRA(config,groundTruthCell,'groundTruth');
%plot results
resultsCell = graphFileToCell(config,strcat(config.groundTruthFileName(1:end-12),'results.graph'));
plotGraphFileICRA(config,resultsCell,'solverResults',...
results.relPose.get('R3xso3Pose'),[])