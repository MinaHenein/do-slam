function [graph] = sortVerticesSimple(graph)
%REORDERVERTICES Summary of this function goes here
%   Detailed explanation goes here

%dont care about reordering

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


end

