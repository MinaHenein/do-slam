function [additionalCost] = compareConstraints(graph,graphC)
%COMPARECONSTRAINTS Summary of this function goes here
%   Detailed explanation goes here

additionalCost = 0;
count = 0;

% identify vertices
%identify types
if (isempty(graph.iPoseVertices) || isempty(graph.iPointVertices))
    graph = graph.identifyTypes;
end
if (isempty(graphC.iPoseVertices) || isempty(graphC.iPointVertices))
    graphC = graphC.identifyTypes;
end
planeVertices = identifyVertices(graphC,'plane');
pointPlaneDistanceError = zeros(1,length(planeVertices));

for i = planeVertices'
    planeVertex = graphC.vertices(i);
    planePoints = graphC.getConnectedVertices(i,'point');
    pointVertices = graphC.iPointVertices;
    
    %point plane error for estimated plane
    constrainedPointPositions = [graphC.vertices(planePoints).value];
    estimatedParameters = planeVertex.value;
    pointPlaneDistancesEstimated = constrainedPointPositions'*estimatedParameters(1:3) - estimatedParameters(4);
    pointPlaneErrorEstimated = norm(pointPlaneDistancesEstimated);
    
    %point plane error for fitted plane    
    unconstrainedPlanePoints = zeros(size(pointVertices)); %location of planePoints in pointVertices;
    for j = 1:numel(unconstrainedPlanePoints)
        if any(planePoints==pointVertices(j))
            unconstrainedPlanePoints(j) = 1;
        end
    end
    unconstrainedPlanePoints = graph.iPointVertices(logical(unconstrainedPlanePoints));
    %get positions of points in unconstrained graph
    unconstrainedPointPositions = [graph.vertices(unconstrainedPlanePoints).value];
    [~,fittedParameters] = fitPlane(unconstrainedPointPositions);
    pointPlaneDistancesFitted = unconstrainedPointPositions'*fittedParameters(1:3) - fittedParameters(4);
    pointPlaneErrorFitted = norm(pointPlaneDistancesFitted);
    
    %error is difference in point-plane distance sums
    count = count+1;
    pointPlaneDistanceError(count) = abs(pointPlaneErrorEstimated - pointPlaneErrorFitted);
    
end

additionalCost = additionalCost + norm(pointPlaneDistanceError);

end

