function [graph,measurementsCell] = adjustAngleConstraints(config,graph,measurementsCell)
%ADJUSTANGLECONSTRAINTS creates fixed angle edges between planes in graph
%   remove existing angle edges
%   identify planes, from current linearisation points, decide which pairs
%   of planes should be linked with angle edges
%   create these edges and associated measurements

%identify angle edges
iAngleEdgesPrev = graph.identifyEdges('plane-plane-fixedAngle');

%split measurements cell into before/after this time step
nPoses = length(graph.identifyVertices('pose'));
measurementsCell1 = measurementsCell(1:graph.nEdges,:);
measurementsCell2 = measurementsCell(graph.nEdges+1:end,:);

%remove angle edges from measurements cell
measurementsCell1(iAngleEdgesPrev,:) = [];

%remove angle edges
oldToNewEdges = ones(1,graph.nEdges);
oldToNewEdges(iAngleEdgesPrev) = 0;
oldToNewEdges = cumsum(oldToNewEdges);
oldToNewEdges(iAngleEdgesPrev) = 0;
%edge indexes
for i = 1:graph.nEdges
    graph.edges(i).index = oldToNewEdges(graph.edges(i).index);
end
graph.edges(iAngleEdgesPrev) = [];
%edge indexes in vertices
for i = 1:graph.nVertices
    graph.vertices(i).iEdges = oldToNewEdges(graph.vertices(i).iEdges);
    graph.vertices(i).iEdges(graph.vertices(i).iEdges==0) = [];
end

%create minimum spanning tree???
%   identify plane vertices
iPlaneVertices = graph.identifyVertices('plane');
nPlaneVertices = numel(iPlaneVertices);
planeAngles = zeros(nPlaneVertices);
for i = 1:nPlaneVertices
    for j = i+1:nPlaneVertices
        normal1 = graph.vertices(iPlaneVertices(i)).value(1:3);
        normal2 = graph.vertices(iPlaneVertices(j)).value(1:3);
        planeAngles(i,j) = abs(dot(normal1,normal2));
    end
end

%create new angle edges
for i = 1:nPlaneVertices
    for j = i+1:nPlaneVertices
        dotProduct = planeAngles(i,j);
        if (dotProduct < 0.1) || (dotProduct > 0.9)
            %measurement
            edgeLabel = config.labelFixedAngleEdge;
            edgeIndex = graph.nEdges+1;
            verticesIn = iPlaneVertices([i j]);
            verticesOut = [];
            edgeValue = dotProduct;
            edgeCovariance = config.covPlanePlaneAngle;
            measurementRow = {edgeLabel,edgeIndex,verticesIn,verticesOut,edgeValue,edgeCovariance};
            measurementsCell1 = vertcat(measurementsCell1,measurementRow);

            %construct edge
            graph = graph.constructFixedAngleEdge(measurementRow);  
        end
    end
end

%rejoin measurementsCell
measurementsCell = vertcat(measurementsCell1,measurementsCell2);

end

