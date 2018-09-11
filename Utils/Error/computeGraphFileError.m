function computeGraphFileError(config,resultsFileName,groundTruthFileName)

resultsCell = graphFileToCell(config,resultsFileName);
groundTruthCell  = graphFileToCell(config,groundTruthFileName);

graphN = Graph(config,resultsCell);
graphGT = Graph(config,groundTruthCell);

results = errorAnalysis(config,graphGT,graphN);

figure('units','normalized','color','w');
xlabel('x (m)')
ylabel('y (m)')
zlabel('z (m)')
hold on
grid on
axis equal
view([-50,25])
plotGraphFileICRA(config,groundTruthCell,'groundTruth');
plotGraphFileICRA(config,resultsCell,'solverResults',...
results.relPose.get('R3xso3Pose'),results.posePointsN.get('R3xso3Pose'))

%plotDynamicPointsTracks(graphN,results.posePointsN.get('R3xso3Pose'))
end