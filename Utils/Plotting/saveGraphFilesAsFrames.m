for i = 1:nSteps+50
    
    if i > nSteps
        deleteMotionVerticesFromGraphFile(strcat(pwd,'/Data/GraphFiles/app6_results_',num2str(nSteps),'.graph'));
        deleteEdgesFromGraphFile(strcat(pwd,'/Data/GraphFiles/app6_results_',num2str(nSteps),'.graph'));
        resultsCell = graphFileToCell(config,strcat('app6_results_',num2str(nSteps),'.graph'));
    else
        deleteMotionVerticesFromGraphFile(strcat(pwd,'/Data/GraphFiles/app6_results_',num2str(i),'.graph'));
        deleteEdgesFromGraphFile(strcat(pwd,'/Data/GraphFiles/app6_results_',num2str(i),'.graph'));
        resultsCell = graphFileToCell(config,strcat('app6_results_',num2str(i),'.graph'));
    end
    
    fig = figure;
    AxesH = axes;
    
    viewPoint = [-35,35];
    axisLimits = [-30,30,0,35,-10,10];
    axis equal
    xlabel('x (m)')
    ylabel('y (m)')
    zlabel('z (m)')
    view(viewPoint)
    axis(axisLimits)
    hold on
    graph = Graph();
    graph = graph.graphFileToGraph(config,resultsCell);
    plotGraphFileICRA(config,resultsCell,'solverResults',zeros(6,1),zeros(6,1),graph,[staticPointsIndices dynamicPointsVertices]);
    hold on
    if i > nSteps && i <= nSteps+25
        plotGraphFile(config,initializationCell,[1 0 0]);
    elseif i > nSteps+25
        plotGraphFile(config,groundTruthCell,[0 1 0]);
    end
    fig.Renderer = 'Painters';
    fig.Position = [20 20 1024 768];
    
    InSet = get(AxesH, 'TightInset');
    set(AxesH, 'Position', [InSet(1:2), 1-InSet(1)-InSet(3), 1-InSet(2)-InSet(4)])
    
    print(fig,'-dpng',strcat('app6_results_',num2str(i),'.png'))
    %print(fig,'-dpdf',strcat('app6_results_',num2str(i),'.pdf'))
    
end