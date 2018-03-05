
function plotTest14(SE3VertexInitialization, noiseLevel, name)
if SE3VertexInitialization == 1
    SE3Vertex = 'Motion Vertex initialized as identity';
elseif SE3VertexInitialization == 2
    SE3Vertex = 'Motion Vertex initialized as translation only';
elseif SE3VertexInitialization == 3
    SE3Vertex = 'Motion Vertex initialized as GT';
end

if noiseLevel == 1
    noise = '[0.01;0.01;0.01]';
elseif noiseLevel == 2
    noise = '[0.05;0.05;0.05]';
end

sequence = 5:16;
newVertexPerNLandmarks = [3,5,8,10];

for i = sequence
    edges = load(strcat('Data/',name,'_results/edges_',num2str(fibonacci(i)),'_',...
        num2str(newVertexPerNLandmarks(end))));
    edges = edges.edges;
end
edges(~any(edges,2),:) = [];

k =[];
edgesCopy = edges(1);
for j = 2:length(edges)
    if edges(j) > edges(j-1)
        edgesCopy(end+1) = edges(j);
    else
        k = [k,j];
    end
end

edges = edgesCopy;

for j = 1:length(newVertexPerNLandmarks)
    nLandmarks = strcat('new motion vertex connected to ',...
        num2str(newVertexPerNLandmarks(j)), ' ', 'landmarks');
    ATE = [];
    ARE = [];
    ASE = [];
    allRTE = [];
    allRRE = [];
    allRSE = [];    
    for i= sequence
        if ~ismember(i,k)
            resultsSE3 = load(strcat('Data/',name,'_results/resultsSE3_',num2str(fibonacci(i)),'_',...
                num2str(newVertexPerNLandmarks(j))));
            resultsSE3 = resultsSE3.resultsSE3;
            resultsNoSE3 = load(strcat('Data/',name,'_results/resultsNoSE3_',num2str(fibonacci(i)),'_',...
                num2str(newVertexPerNLandmarks(j))));
            resultsNoSE3 = resultsNoSE3.resultsNoSE3;
            
            ATE = [ATE, resultsNoSE3.ATE_translation_error - resultsSE3.ATE_translation_error];
            ARE = [ARE, resultsNoSE3.ATE_rotation_error - resultsSE3.ATE_rotation_error];
            ASE = [ASE, resultsNoSE3.ASE_translation_error - resultsSE3.ASE_translation_error];
            
            allRTE = [allRTE, resultsNoSE3.AARPE_translation_error - resultsSE3.AARPE_translation_error];
            allRRE = [allRRE, resultsNoSE3.AARPE_rotation_error - resultsSE3.AARPE_rotation_error];
            allRSE = [allRSE, resultsNoSE3.AARPTE_translation_error - resultsSE3.AARPTE_translation_error];
        end
    end
    
    figure
    yyaxis left
    plot(edges,ATE,'-bo')
    hold on;
    plot(edges,ASE,'-gs')
    
    yyaxis right
    plot(edges,ARE,'-r*')
    
    yyaxis left
    title({SE3Vertex,strcat('Ternary factory noise level: ',noise),nLandmarks});
    xlabel('nMotionEdges')
    ylabel('differences of absolute translational error values')
    legend('w/o DOM ATE - w/ DOM ATE','w/o DOM ASE - w/ DOM ASE','w/o DOM ARE - w/ DOM ARE','Location','northwest')
    yyaxis right
    ylabel('differences of  absolute rotational error values')
    print(strcat('Absolute_',name,'_',num2str(newVertexPerNLandmarks(j))),'-dpdf')
    
    figure
    yyaxis left
    plot(edges,allRTE,'-bo')
    hold on;
    plot(edges,allRSE,'-gs')
    
    yyaxis right
    plot(edges,allRRE,'-r*')
    
    yyaxis left
    title({SE3Vertex,strcat('Ternary factory noise level: ',noise),nLandmarks});
    xlabel('nMotionEdges')
    ylabel('differences of all-all relative translational error values')
    legend('w/o DOM allRTE - w/ DOM allRTE','w/o DOM allRSE - w/ DOM allRSE','w/o DOM allRRE - w/ DOM allRRE','Location','northwest')
    yyaxis right
    ylabel('differences of all-all relative rotational error values')
    print(strcat('Relative_',name,'_',num2str(newVertexPerNLandmarks(j))),'-dpdf')
    
end

end