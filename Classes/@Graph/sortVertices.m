function [graph,newToOldMap,measurementsCellSorted] = sortVertices(graph,measurementsCell)
%REORDERVERTICES Summary of this function goes here
%   Detailed explanation goes here

%identify indexes
iPoseVertices  = graph.identifyVertices('pose');
iPointVertices = graph.identifyVertices('point');
iPlaneVertices = graph.identifyVertices('plane');

newToOldMap = [iPoseVertices; iPointVertices; iPlaneVertices];
oldToNewMap = zeros(size(newToOldMap));
for i = 1:graph.nVertices
    oldToNewMap(newToOldMap(i)) = i;
end

%preallocate
verticesReordered = Vertex();
verticesReordered(1:graph.nVertices) = Vertex();

%reorder vertices
for i = 1:graph.nVertices
    verticesReordered(i) = graph.vertices(newToOldMap(i));
    verticesReordered(i).index = i;
end
graph.vertices = verticesReordered;

%map vertex indexes in edges
measurementsCellSorted = measurementsCell;
for i = 1:graph.nEdges
    graph.edges(i).iVertices = oldToNewMap(graph.edges(i).iVertices)';

    %map vertices in measurementsCell from 1:graph.nEdges
    measurementsCellSorted{i,3} = oldToNewMap(measurementsCell{i,3}); 
    measurementsCellSorted{i,4} = oldToNewMap(measurementsCell{i,4});
end


end

